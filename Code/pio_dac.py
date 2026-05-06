"""
PIO + DMA MCP4922 DAC Driver — Raspberry Pi Pico 2 W (RP2350)

Generates continuous sine waves via PIO bit-bang SPI + DMA, with zero CPU
load. Two independent DACs produce temporal interference beat frequencies.

Pin assignments:
    DAC 1: GPIO 16 (LDAC), 17 (CS), 18 (SCK), 19 (MOSI)
    DAC 2: GPIO 10 (LDAC), 11 (CS), 12 (SCK), 13 (MOSI)
    Shared: GPIO 20 (SHDN) — hardware shutdown for both DACs

PIO pin mapping (per state machine):
    SET pin 0  = LDAC
    SET pin 1  = CS     (LDAC + 1)
    side-set   = SCK    (LDAC + 2)
    OUT        = MOSI   (LDAC + 3)

IMPORTANT: If you've been toggling SPI pins manually with Pin(),
do a soft reset (Ctrl+D in REPL) before running this script.
The PIO needs to own those pins.
"""

import rp2
from machine import Pin, Timer, mem32
import micropython
import time
import math
import array
import uctypes


# ============================================================
# PIO Assembly: SPI Mode 0,0 for MCP4922 (fast, 38 cycles/sample)
# ============================================================
#
# 2 cycles per bit: SCK-high shares a cycle with the branch.
# SCK high time = 1 PIO cycle — within MCP4922 specs at 3.3V.
#
# Cycles: 3 setup + 32 bit-bang (16 x 2) + 3 teardown = 38
#
# SET encoding:
#   0b11 = CS high, LDAC high  (idle)
#   0b01 = CS low,  LDAC high  (SPI active)
#   0b10 = CS high, LDAC low   (latch pulse)

@rp2.asm_pio(
    sideset_init=rp2.PIO.OUT_LOW,
    out_init=rp2.PIO.OUT_LOW,
    set_init=(rp2.PIO.OUT_HIGH, rp2.PIO.OUT_HIGH),
    out_shiftdir=rp2.PIO.SHIFT_LEFT,
    autopull=False,
)
def mcp4922_spi():
    pull(block)              .side(0)
    set(pins, 0b01)          .side(0)
    set(x, 15)               .side(0)
    label("bitloop")
    out(pins, 1)             .side(0)
    jmp(x_dec, "bitloop")    .side(1)
    set(pins, 0b11)          .side(0)
    set(pins, 0b10)          .side(0)
    set(pins, 0b11)          .side(0)


CYCLES_PER_SAMPLE = 38
CHANNEL_A = 0
CHANNEL_B = 1
DMA_BASE = 0x50000000


# ============================================================
# Command word helpers
# ============================================================

def build_cmd(channel, value, gain_1x=True, buffered=False):
    """Build a 32-bit PIO-ready command word for the MCP4922.

    Returns the 16-bit SPI command left-aligned in a 32-bit int,
    ready to push directly into the PIO TX FIFO.
    """
    cmd = (channel & 1) << 15
    cmd |= (1 if buffered else 0) << 14
    cmd |= (1 if gain_1x else 0) << 13
    cmd |= 1 << 12
    cmd |= value & 0x0FFF
    return cmd << 16


def voltage_to_cmd(channel, voltage, vref=3.3, gain_1x=True):
    """Convert a target voltage to a PIO-ready command word."""
    gain = 1 if gain_1x else 2
    value = int((voltage * 4096) / (vref * gain))
    value = max(0, min(4095, value))
    return build_cmd(channel, value, gain_1x)


def build_sine_lut(channel, lut_size=256, amplitude_v=1.65, center_v=1.65,
                   vref=3.3):
    """Pre-compute a sine wave LUT as PIO-ready 32-bit command words.

    Args:
        channel: CHANNEL_A or CHANNEL_B
        lut_size: number of samples per cycle
        amplitude_v: peak amplitude in volts (swing is center +/- amplitude)
        center_v: DC offset in volts (default 1.65V = mid-scale)
        vref: DAC reference voltage (default 3.3V)
    """
    lut = array.array('I', [0] * lut_size)
    for i in range(lut_size):
        v = center_v + amplitude_v * math.sin(2 * math.pi * i / lut_size)
        dac_val = int((v / vref) * 4096)
        dac_val = max(0, min(4095, dac_val))
        lut[i] = build_cmd(channel, dac_val)
    return lut


# ============================================================
# PIO DAC driver with DMA support
# ============================================================

class PIODac:
    """MCP4922 driver: PIO for SPI, DMA for continuous waveforms.

    Pins must be consecutive: LDAC, CS (LDAC+1), SCK (LDAC+2), MOSI (LDAC+3).
    This is required by PIO SET/side-set/OUT pin mapping.

    Args:
        sm_id: PIO state machine number (0-7)
        ldac_pin: GPIO number for LDAC (CS=ldac+1, SCK=ldac+2, MOSI=ldac+3)
        shdn_pin: GPIO number for SHDN (shared across DACs, or None)
    """

    def __init__(self, sm_id=0, ldac_pin=16, shdn_pin=20):
        if shdn_pin is not None:
            self._shdn = Pin(shdn_pin, Pin.OUT, value=1)
        else:
            self._shdn = None

        self._sm = rp2.StateMachine(
            sm_id,
            mcp4922_spi,
            freq=10_000_000,
            set_base=Pin(ldac_pin),
            sideset_base=Pin(ldac_pin + 2),
            out_base=Pin(ldac_pin + 3),
        )
        self._sm.active(1)
        self._pio_freq = 10_000_000
        self._sm_id = sm_id
        self._ldac_pin = ldac_pin

        self._dma_main = None
        self._dma_loop = None
        self._lut = None
        self._addr_buf = None
        self._waveform_active = False

        self._current_amp = 0.0
        self._current_center = 1.65
        self._current_channel = CHANNEL_A
        self._vref = 3.3

        pio_num = sm_id // 4
        sm_num = sm_id % 4
        pio_base = 0x50200000 + pio_num * 0x100000
        self._txf_addr = pio_base + 0x10 + sm_num * 4
        self._dreq_tx = pio_num * 8 + sm_num

        self.write(build_cmd(CHANNEL_A, 0))
        time.sleep_us(10)
        print(f"PIO DAC ready: SM{sm_id} pins {ldac_pin}-{ldac_pin+3}")

    def write(self, cmd32):
        """Push a pre-built 32-bit command into the PIO FIFO."""
        self._sm.put(cmd32)

    def set_voltage(self, channel, voltage, vref=3.3):
        """Set a channel to a specific voltage."""
        self.write(voltage_to_cmd(channel, voltage, vref))

    # ---- Frequency control ----

    @staticmethod
    def _quantize_clkdiv(sys_clk, desired_pio_freq):
        """Compute the actual PIO frequency after CLKDIV register quantization.

        The CLKDIV register has 16-bit integer + 8-bit fractional (1/256)
        divider. Returns (int_div, frac_div, actual_pio_freq).
        """
        div = sys_clk / desired_pio_freq
        int_div = int(div)
        frac_div = int((div - int_div) * 256)

        if int_div < 1:
            int_div, frac_div = 1, 0
        elif int_div > 65535:
            int_div, frac_div = 65535, 0

        actual_div = int_div + frac_div / 256
        actual_pio_freq = sys_clk / actual_div
        return int_div, frac_div, actual_pio_freq

    def actual_output_freq(self, desired_freq, lut_size=256):
        """Compute the actual output frequency after CLKDIV quantization."""
        import machine
        sys_clk = machine.freq()
        desired_pio = desired_freq * lut_size * CYCLES_PER_SAMPLE
        _, _, actual_pio = self._quantize_clkdiv(sys_clk, desired_pio)
        return actual_pio / (CYCLES_PER_SAMPLE * lut_size)

    def set_pio_freq(self, freq):
        """Change PIO clock by writing the CLKDIV register directly.

        Non-disruptive: SM keeps running, DMA keeps feeding.
        Returns the actual PIO frequency achieved after quantization.
        """
        import machine
        sys_clk = machine.freq()
        int_div, frac_div, actual = self._quantize_clkdiv(sys_clk, freq)

        pio_num = self._sm_id // 4
        sm_num = self._sm_id % 4
        pio_base = 0x50200000 + pio_num * 0x100000
        clkdiv_addr = pio_base + 0x0C8 + sm_num * 0x18

        mem32[clkdiv_addr] = (int_div << 16) | (frac_div << 8)
        self._pio_freq = actual
        return actual

    def set_frequency(self, freq, lut_size=None):
        """Change waveform frequency on the fly (adjusts PIO clock only).

        Does not rebuild the LUT — the waveform shape stays the same.
        """
        if lut_size is None:
            lut_size = len(self._lut) if self._lut else 256
        pio_freq = int(freq * lut_size * CYCLES_PER_SAMPLE)
        sck_freq = pio_freq / 2
        if sck_freq > 20_000_000:
            raise ValueError(
                f"SCK would be {sck_freq/1e6:.1f} MHz (max 20). "
                f"Reduce lut_size or freq."
            )
        self.set_pio_freq(pio_freq)

    # ---- Output control ----

    def zero(self):
        """Zero both channels."""
        self.write(build_cmd(CHANNEL_A, 0))
        time.sleep_us(100)
        self.write(build_cmd(CHANNEL_B, 0))
        time.sleep_us(100)

    def shutdown(self):
        if self._shdn:
            self._shdn.value(0)

    def wake(self):
        if self._shdn:
            self._shdn.value(1)

    # ---- DMA waveform engine ----

    def start_sine(self, channel, freq, lut_size=256, amplitude_v=1.65,
                   center_v=1.65):
        """Start a continuous DMA-driven sine wave. CPU is free after this.

        Args:
            channel: CHANNEL_A (0) or CHANNEL_B (1)
            freq: Waveform frequency in Hz
            lut_size: Samples per cycle (more = smoother, fewer = higher max freq)
            amplitude_v: Peak amplitude in volts (swing is center_v +/- amplitude_v)
            center_v: DC center voltage (default 1.65V = mid-scale)
        """
        self.stop_waveform()
        self._lut = build_sine_lut(channel, lut_size, amplitude_v, center_v)

        pio_freq = int(freq * lut_size * CYCLES_PER_SAMPLE)
        sck_freq = pio_freq / 2
        if sck_freq > 20_000_000:
            raise ValueError(
                f"SCK would be {sck_freq/1e6:.1f} MHz (max 20). "
                f"Reduce lut_size or freq."
            )
        actual_pio = self.set_pio_freq(pio_freq)
        self._start_dma_loop(lut_size)
        self._waveform_active = True

        self._current_amp = amplitude_v
        self._current_center = center_v
        self._current_channel = channel

        actual = actual_pio / (CYCLES_PER_SAMPLE * lut_size)
        print(f"DMA sine: requested {freq:.1f} Hz, actual {actual:.3f} Hz, "
              f"{lut_size} pts, SCK {sck_freq/1e6:.1f} MHz")

    def rewrite_lut_amplitude(self, amplitude_v, center_v=None):
        """Rewrite the running LUT in place at a new amplitude / center.

        Safe to call while DMA is streaming — the DMA reads from the same
        memory and will pick up new samples on its next pass. Used by the
        amplitude ramp engine to fade waveforms in and out without stopping
        DMA. No allocation: writes back into the existing array.
        """
        if not self._waveform_active or self._lut is None:
            return
        if center_v is None:
            center_v = self._current_center
        lut_size = len(self._lut)
        channel = self._current_channel
        vref = self._vref
        two_pi = 2 * math.pi
        for i in range(lut_size):
            v = center_v + amplitude_v * math.sin(two_pi * i / lut_size)
            dac_val = int((v / vref) * 4096)
            if dac_val < 0:
                dac_val = 0
            elif dac_val > 4095:
                dac_val = 4095
            self._lut[i] = build_cmd(channel, dac_val)
        self._current_amp = amplitude_v
        self._current_center = center_v

    def _start_dma_loop(self, count):
        """Set up two chained DMA channels for infinite LUT playback.

        Main DMA: LUT -> PIO TX FIFO, paced by PIO DREQ.
        Loop DMA: resets Main's read pointer and re-triggers it.
        """
        self._dma_main = rp2.DMA()
        self._dma_loop = rp2.DMA()

        self._addr_buf = array.array('I', [uctypes.addressof(self._lut)])

        main_ctrl = self._dma_main.pack_ctrl(
            size=2,
            inc_read=True,
            inc_write=False,
            treq_sel=self._dreq_tx,
            chain_to=self._dma_loop.channel,
        )

        main_read_trig = DMA_BASE + self._dma_main.channel * 0x40 + 0x3C
        loop_ctrl = self._dma_loop.pack_ctrl(
            size=2,
            inc_read=False,
            inc_write=False,
            treq_sel=0x3F,
            chain_to=self._dma_loop.channel,
        )

        self._dma_loop.config(
            read=self._addr_buf,
            write=main_read_trig,
            count=1,
            ctrl=loop_ctrl,
            trigger=False,
        )

        self._dma_main.config(
            read=self._lut,
            write=self._txf_addr,
            count=count,
            ctrl=main_ctrl,
            trigger=True,
        )

    def stop_waveform(self):
        """Stop DMA waveform and zero the output."""
        if self._dma_main:
            self._dma_main.active(0)
        if self._dma_loop:
            self._dma_loop.active(0)

        if self._waveform_active:
            self._sm.active(0)
            self._sm.active(1)
            self.write(build_cmd(CHANNEL_A, 0))
            time.sleep_us(100)
            self._waveform_active = False

        if self._dma_main:
            self._dma_main.close()
            self._dma_main = None
        if self._dma_loop:
            self._dma_loop.close()
            self._dma_loop = None

    def deinit(self):
        """Full cleanup: stop waveform, zero outputs, release SM."""
        self.stop_waveform()
        self.zero()
        self._sm.active(0)


# ============================================================
# Dual DAC driver for temporal interference
# ============================================================

DAC1_LDAC_PIN = 16
DAC2_LDAC_PIN = 10
SHDN_PIN = 20


class DualDac:
    """Two independent MCP4922 DACs for temporal interference stimulation.

    Each DAC has its own PIO state machine and DMA loop, running at
    independent frequencies. The beat frequency perceived by neurons
    is |f1 - f2|.
    """

    def __init__(self):
        self.dac1 = PIODac(sm_id=0, ldac_pin=DAC1_LDAC_PIN, shdn_pin=SHDN_PIN)
        self.dac2 = PIODac(sm_id=1, ldac_pin=DAC2_LDAC_PIN, shdn_pin=None)
        self._shdn = self.dac1._shdn

        self._ramp_timer = None
        self._ramp_step = 0
        self._ramp_total_steps = 0
        self._ramp_start_a1 = 0.0
        self._ramp_start_a2 = 0.0
        self._ramp_target_a1 = 0.0
        self._ramp_target_a2 = 0.0
        self._ramp_finish_cb = None
        self._ramp_step_ref = self._ramp_step_cb
        self._ramp_tick_ref = self._ramp_tick

    def _find_best_lut(self, f1, f2, min_lut=16, max_lut=256):
        """Find the LUT size that minimizes beat frequency error.

        Tries power-of-2 LUT sizes and picks the one where the CLKDIV
        quantization of both frequencies preserves the requested
        beat = |f1 - f2| most accurately.

        Returns (best_lut_size, actual_f1, actual_f2, actual_beat).
        """
        target_beat = abs(f1 - f2)
        best = None
        lut = min_lut
        while lut <= max_lut:
            a1 = self.dac1.actual_output_freq(f1, lut)
            a2 = self.dac2.actual_output_freq(f2, lut)
            actual_beat = abs(a1 - a2)
            err = abs(actual_beat - target_beat)
            if best is None or err < best[0]:
                best = (err, lut, a1, a2, actual_beat)
            lut *= 2
        _, best_lut, af1, af2, ab = best
        return best_lut, af1, af2, ab

    def start_temporal_interference(self, f1, f2, channel=CHANNEL_A,
                                    lut_size=None, amplitude_v=1.65,
                                    center_v=1.65):
        """Start two sine waves for temporal interference.

        Args:
            f1: Frequency for DAC 1 (Hz)
            f2: Frequency for DAC 2 (Hz)
            channel: Which MCP4922 channel on each DAC (A or B)
            lut_size: Samples per sine cycle (None = auto-optimize for
                      best beat accuracy)
            amplitude_v: Peak amplitude in volts (swing is center_v +/- amplitude_v)
            center_v: DC center voltage (default 1.65V = mid-scale)

        Beat frequency = |f1 - f2|.
        """
        if lut_size is None:
            lut_size, af1, af2, ab = self._find_best_lut(f1, f2)
        else:
            af1 = self.dac1.actual_output_freq(f1, lut_size)
            af2 = self.dac2.actual_output_freq(f2, lut_size)
            ab = abs(af1 - af2)

        self.dac1.start_sine(channel, f1, lut_size, amplitude_v, center_v)
        self.dac2.start_sine(channel, f2, lut_size, amplitude_v, center_v)

        beat = abs(f1 - f2)
        center = (f1 + f2) / 2
        print(f"Temporal interference: {f1} Hz + {f2} Hz "
              f"-> {beat} Hz beat (center {center:.0f} Hz)")
        print(f"  Voltage: {center_v}V center, {amplitude_v}V amplitude "
              f"({center_v - amplitude_v:.3f}V - {center_v + amplitude_v:.3f}V)")
        print(f"  Actual: {af1:.3f} Hz + {af2:.3f} Hz "
              f"-> {ab:.3f} Hz beat (LUT {lut_size} pts)")
        if abs(ab - beat) > 0.1:
            print(f"  WARNING: beat error = {abs(ab - beat):.3f} Hz")

    def set_beat_frequency(self, center_freq, beat_freq, lut_size=None,
                           amplitude_v=1.65, center_v=1.65):
        """Adjust frequencies to achieve a target beat at a given center.

        If a waveform is already running, changes frequency on the fly
        (PIO clock divider only, no DMA restart). If not running, starts
        new sine waves.
        """
        f1 = center_freq - beat_freq / 2
        f2 = center_freq + beat_freq / 2

        if lut_size is None:
            lut_size, af1, af2, ab = self._find_best_lut(f1, f2)
        else:
            af1 = self.dac1.actual_output_freq(f1, lut_size)
            af2 = self.dac2.actual_output_freq(f2, lut_size)
            ab = abs(af1 - af2)

        if self.dac1._waveform_active and self.dac2._waveform_active:
            self.dac1.set_frequency(f1, lut_size)
            self.dac2.set_frequency(f2, lut_size)
        else:
            self.dac1.start_sine(CHANNEL_A, f1, lut_size, amplitude_v, center_v)
            self.dac2.start_sine(CHANNEL_A, f2, lut_size, amplitude_v, center_v)

        print(f"Beat: {f1:.1f} Hz + {f2:.1f} Hz -> {beat_freq} Hz "
              f"(center {center_freq} Hz)")
        print(f"  Actual: {af1:.3f} Hz + {af2:.3f} Hz "
              f"-> {ab:.3f} Hz beat (LUT {lut_size} pts)")

    # ---- Amplitude ramp engine ----
    #
    # A virtual Timer fires `steps` times across `duration_s`. The hard-IRQ
    # callback (_ramp_tick) does as little as possible — it just defers the
    # real LUT rewrite onto micropython.schedule, which runs in the regular
    # scheduler context where allocation is safe. Each scheduled step
    # linearly interpolates each DAC's amplitude between its start and
    # target values and rewrites the live LUT in place.

    def _cancel_ramp(self):
        """Stop any in-flight ramp timer. Safe to call when no ramp active."""
        t = self._ramp_timer
        self._ramp_timer = None
        self._ramp_finish_cb = None
        if t is not None:
            try:
                t.deinit()
            except Exception as e:
                print("ramp timer deinit failed:", e)

    def _start_ramp(self, target1, target2, duration_s, steps=20,
                    on_finish=None):
        """Begin a non-blocking amplitude ramp on both DACs.

        Args:
            target1: final amplitude (V) for DAC 1
            target2: final amplitude (V) for DAC 2
            duration_s: total ramp time in seconds
            steps: number of LUT updates across the ramp
            on_finish: optional callable invoked after the last step
        """
        self._cancel_ramp()
        if steps < 1:
            steps = 1
        self._ramp_step = 0
        self._ramp_total_steps = steps
        self._ramp_start_a1 = self.dac1._current_amp
        self._ramp_start_a2 = self.dac2._current_amp
        self._ramp_target_a1 = target1
        self._ramp_target_a2 = target2
        self._ramp_finish_cb = on_finish

        period_ms = int(duration_s * 1000 / steps)
        if period_ms < 1:
            period_ms = 1

        self._ramp_timer = Timer(-1)
        self._ramp_timer.init(
            period=period_ms,
            mode=Timer.PERIODIC,
            callback=self._ramp_tick_ref,
        )

    def _ramp_tick(self, _t):
        """Hard-IRQ callback: defer real work to the scheduler context."""
        try:
            micropython.schedule(self._ramp_step_ref, 0)
        except RuntimeError:
            # schedule queue full — skip this tick, next one will catch up
            pass

    def _ramp_step_cb(self, _):
        """Scheduler-context callback: advance one step of the ramp."""
        if self._ramp_timer is None:
            return
        self._ramp_step += 1
        progress = self._ramp_step / self._ramp_total_steps
        if progress > 1.0:
            progress = 1.0

        amp1 = (self._ramp_start_a1
                + (self._ramp_target_a1 - self._ramp_start_a1) * progress)
        amp2 = (self._ramp_start_a2
                + (self._ramp_target_a2 - self._ramp_start_a2) * progress)
        self.dac1.rewrite_lut_amplitude(amp1)
        self.dac2.rewrite_lut_amplitude(amp2)

        if self._ramp_step >= self._ramp_total_steps:
            cb = self._ramp_finish_cb
            self._cancel_ramp()
            if cb is not None:
                try:
                    cb()
                except Exception as e:
                    print("ramp finish callback error:", e)

    # ---- Ramp wrappers ----

    def start_temporal_interference_ramp(self, f1, f2, target_amp_v,
                                         center_v=1.65,
                                         duration_s=1.0,
                                         channel=CHANNEL_A,
                                         lut_size=None,
                                         steps=20):
        """Start TI with amplitude ramping smoothly from 0 to target_amp_v.

        DMA is started immediately at amplitude 0 (flat-line LUT at the
        center voltage), then a non-blocking ramp fades the carrier in.
        """
        self._cancel_ramp()
        self.start_temporal_interference(
            f1, f2,
            channel=channel,
            lut_size=lut_size,
            amplitude_v=0.0,
            center_v=center_v,
        )
        self._start_ramp(target_amp_v, target_amp_v, duration_s, steps=steps)

    def set_beat_frequency_ramp(self, center_freq, beat_freq, target_amp_v,
                                center_v=1.65, duration_s=1.0,
                                lut_size=None, steps=20):
        """Set beat frequencies with a smooth amplitude ramp from 0."""
        self._cancel_ramp()
        self.set_beat_frequency(
            center_freq, beat_freq,
            lut_size=lut_size,
            amplitude_v=0.0,
            center_v=center_v,
        )
        self._start_ramp(target_amp_v, target_amp_v, duration_s, steps=steps)

    def start_sine_ramp(self, dac, channel, freq, target_amp_v,
                        center_v=1.65, duration_s=1.0,
                        lut_size=256, steps=20):
        """Start a single-DAC sine with a smooth amplitude ramp from 0.

        `dac` is the PIODac instance (self.dac1 or self.dac2). The other
        DAC's target is kept at its current amplitude so the ramp engine
        only fades the requested DAC.
        """
        self._cancel_ramp()
        dac.start_sine(channel, freq, lut_size,
                       amplitude_v=0.0, center_v=center_v)
        if dac is self.dac1:
            t1, t2 = target_amp_v, self.dac2._current_amp
        else:
            t1, t2 = self.dac1._current_amp, target_amp_v
        self._start_ramp(t1, t2, duration_s, steps=steps)

    def ramp_stop(self, duration_s=1.0, steps=20):
        """Smoothly ramp both DACs to 0 V amplitude, then stop DMA.

        If no waveform is currently active, falls through to an immediate
        stop so callers can use this unconditionally.
        """
        if not (self.dac1._waveform_active or self.dac2._waveform_active):
            self.stop()
            return
        self._start_ramp(0.0, 0.0, duration_s, steps=steps,
                         on_finish=self.stop)

    def stop(self):
        """Stop both DAC waveforms and zero outputs."""
        self._cancel_ramp()
        self.dac1.stop_waveform()
        self.dac2.stop_waveform()

    def emergency_stop(self):
        """Kill everything: cancel ramp, kill DMA, pull SHDN low."""
        self._cancel_ramp()
        self.dac1.stop_waveform()
        self.dac2.stop_waveform()
        if self._shdn:
            self._shdn.value(0)
        print("EMERGENCY STOP - all outputs killed")

    def deinit(self):
        self.stop()
        self.dac1.deinit()
        self.dac2.deinit()


# ============================================================
# Tests
# ============================================================

def test_dual_dc():
    """Set known voltages on both DACs to verify wiring."""
    dual = DualDac()

    test_points = [
        (0,    "0.000V"),
        (1024, "0.825V"),
        (2048, "1.650V"),
        (3072, "2.475V"),
        (4095, "3.299V"),
    ]

    print("\n" + "=" * 50)
    print("Test: Dual DAC DC Voltages")
    print("=" * 50)

    try:
        print("\n--- DAC 1 Channel A ---")
        for value, expected in test_points:
            dual.dac1.write(build_cmd(CHANNEL_A, value))
            voltage = 3.3 * value / 4096
            print(f"\n  DAC1 A: DAC={value:4d}  Expected={expected}  "
                  f"Calc={voltage:.3f}V")
            print(f"  -> Measure DAC 1 VOUTA, press Enter...")
            input()

        dual.dac1.write(build_cmd(CHANNEL_A, 0))
        time.sleep_us(100)

        print("\n--- DAC 2 Channel A ---")
        for value, expected in test_points:
            dual.dac2.write(build_cmd(CHANNEL_A, value))
            voltage = 3.3 * value / 4096
            print(f"\n  DAC2 A: DAC={value:4d}  Expected={expected}  "
                  f"Calc={voltage:.3f}V")
            print(f"  -> Measure DAC 2 VOUTA, press Enter...")
            input()

        print("\n--- Both DACs simultaneously ---")
        dual.dac1.write(build_cmd(CHANNEL_A, 2048))
        dual.dac2.write(build_cmd(CHANNEL_A, 4095))
        print("\n  DAC1=1.65V, DAC2=3.3V — verify both, press Enter...")
        input()

        dual.dac1.write(build_cmd(CHANNEL_A, 4095))
        dual.dac2.write(build_cmd(CHANNEL_A, 2048))
        print("  DAC1=3.3V, DAC2=1.65V — verify both, press Enter...")
        input()

    finally:
        dual.deinit()
    print("DC test complete.")


def test_temporal_interference():
    """Run two sine waves at slightly different frequencies."""
    dual = DualDac()

    print("\n" + "=" * 50)
    print("Test: Temporal Interference — ~7.25 kHz, 10 Hz beat")
    print("=" * 50)

    try:
        f1, f2, lut = 7250, 7260, 64
        print(f"\n  Target: {f1} Hz + {f2} Hz -> 10 Hz beat  (LUT={lut})")
        dual.start_temporal_interference(f1, f2, lut_size=lut,
                                         amplitude_v=1.0, center_v=1.0)

        print("\n  Running. Check scope: CH1+CH2 should show 10 Hz envelope.")
        print("  Ctrl+C to stop.")
        while True:
            time.sleep(1)

    except KeyboardInterrupt:
        print("\n  Stopped")

    dual.emergency_stop()
    dual.deinit()
    print("\nTemporal interference test complete.")


if __name__ == "__main__":
    # test_dual_dc()
    test_temporal_interference()
