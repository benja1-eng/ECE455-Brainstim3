"""
BLE Wave Generator — Raspberry Pi Pico 2 W

Advertises as "PicoW_DAC" and exposes a single writable GATT characteristic
that accepts pipe-separated text commands. Wraps the DualDac driver from
pio_dac.py and drives two MCP4922 DACs with PIO+DMA sine waves.

Upload as `main.py` alongside `pio_dac.py` on the Pico, or run manually in
REPL. A soft reset (Ctrl+D) is recommended before running if you've been
poking SPI pins by hand, so the PIO can take ownership.

Command protocol (UTF-8 text, pipe separated):

    TI|f1|f2|amp|center            Two sines, separate DACs (temporal interference)
    BEAT|center|beat|amp|center_v  Same, expressed as center + beat frequency
    UPDATE|f1|f2                   On-the-fly freq change (no DMA restart)
    SINE1|f|amp|center             Single sine on DAC 1, channel A
    SINE2|f|amp|center             Single sine on DAC 2, channel A
    STOP                           Graceful stop, zero outputs
    ESTOP                          Emergency stop + hardware SHDN low
    WAKE                           Release hardware SHDN
    ZERO                           Zero both DAC outputs (keep DMA off)
    PING                           Reply with "PONG" via notify

Replies (sent as GATT notifications on the same characteristic):

    OK|<echo of command>           Command accepted and executed
    ERR|<message>                  Command rejected or raised an exception
    STATE|running=<0|1>|f1=<..>|f2=<..>|beat=<..>|amp=<..>|center=<..>

All numeric values are floats in Hz (frequencies) or volts (amp / center).
"""

import bluetooth
import struct
import time
from micropython import const

from pio_dac import DualDac, CHANNEL_A


# ----------------------------------------------------------------------
# BLE constants
# ----------------------------------------------------------------------

_SERVICE_UUID = bluetooth.UUID(0x1815)   # Automation IO (matches old_testing)
_CHAR_UUID    = bluetooth.UUID(0x2A56)   # Digital characteristic

_IRQ_CENTRAL_CONNECT    = const(1)
_IRQ_CENTRAL_DISCONNECT = const(2)
_IRQ_GATTS_WRITE        = const(3)

_FLAG_READ   = bluetooth.FLAG_READ
_FLAG_WRITE  = bluetooth.FLAG_WRITE
_FLAG_NOTIFY = bluetooth.FLAG_NOTIFY


# Default amplitude ramp duration applied to TI / BEAT / SINE / STOP.
# ESTOP and BLE-disconnect stops bypass this and zero outputs immediately.
RAMP_DURATION_S = 1.0

# Samples per sine cycle for BLE-driven waveforms. 128 = 2.81 deg / step
# (visually clean on a scope) and supports carriers up to ~8.2 kHz under
# the MCP4922's 20 MHz SCK ceiling.
LUT_SIZE = 128


# ----------------------------------------------------------------------
# DAC state — updated by command handlers, reported via STATE replies
# ----------------------------------------------------------------------

class DacState:
    def __init__(self):
        self.running = False
        self.f1 = 0.0
        self.f2 = 0.0
        self.amp = 1.0
        self.center = 1.65

    @property
    def beat(self):
        return abs(self.f1 - self.f2)

    def as_reply(self):
        return ("STATE|running={r}|f1={f1:.3f}|f2={f2:.3f}|beat={b:.3f}"
                "|amp={a:.3f}|center={c:.3f}").format(
                    r=1 if self.running else 0,
                    f1=self.f1, f2=self.f2, b=self.beat,
                    a=self.amp, c=self.center)


# ----------------------------------------------------------------------
# BLE wrapper
# ----------------------------------------------------------------------

class BLEDacController:
    def __init__(self, dual, name="PicoW_DAC"):
        self._dual = dual
        self._state = DacState()

        self._ble = bluetooth.BLE()
        self._ble.active(True)
        self._ble.irq(self._irq)
        self._connections = set()
        self._name = name

        char_flags = _FLAG_READ | _FLAG_WRITE | _FLAG_NOTIFY
        services = ((_SERVICE_UUID, ((_CHAR_UUID, char_flags),)),)
        ((self._handle,),) = self._ble.gatts_register_services(services)

        self._advertise()

    # -- advertising -----------------------------------------------------

    def _advertise(self):
        name = self._name.encode()
        adv = bytearray()
        adv.extend(struct.pack('BB', 2, 0x01))           # flags
        adv.extend(struct.pack('B', 0x06))               # LE general disc, BR/EDR off
        adv.extend(struct.pack('BB', len(name) + 1, 0x09))  # complete local name
        adv.extend(name)
        adv.extend(struct.pack('BB', 3, 0x03))           # complete list 16-bit UUIDs
        adv.extend(struct.pack('<H', 0x1815))
        self._ble.gap_advertise(100_000, adv)
        print("BLE advertising as '{}'".format(self._name))

    # -- IRQ -------------------------------------------------------------

    def _irq(self, event, data):
        if event == _IRQ_CENTRAL_CONNECT:
            conn_handle, _, _ = data
            self._connections.add(conn_handle)
            print("Central connected:", conn_handle)

        elif event == _IRQ_CENTRAL_DISCONNECT:
            conn_handle, _, _ = data
            self._connections.discard(conn_handle)
            print("Central disconnected:", conn_handle)
            # Safety: any disconnect stops the waveform. A dropped link
            # should never leave a stimulation signal running.
            self._safe_stop()
            self._advertise()

        elif event == _IRQ_GATTS_WRITE:
            conn_handle, value_handle = data
            if value_handle != self._handle:
                return
            raw = self._ble.gatts_read(self._handle)
            try:
                cmd = raw.decode('utf-8').strip()
            except Exception as e:
                self._notify("ERR|decode:{}".format(e))
                return
            if not cmd:
                return
            print("cmd:", cmd)
            self._handle_command(cmd)

    # -- notify helpers --------------------------------------------------

    def _notify(self, text):
        data = text.encode('utf-8')
        self._ble.gatts_write(self._handle, data)
        for conn in tuple(self._connections):
            try:
                self._ble.gatts_notify(conn, self._handle, data)
            except Exception as e:
                print("notify failed:", e)

    def _notify_state(self):
        self._notify(self._state.as_reply())

    # -- command dispatch ------------------------------------------------

    def _handle_command(self, cmd):
        try:
            parts = cmd.split('|')
            op = parts[0].upper()
            args = parts[1:]

            if op == "PING":
                self._notify("PONG")
                return

            elif op == "STATUS":
                self._notify_state()
                return

            elif op == "TI":
                vals, dur = self._parse_floats_opt_duration(args, 4)
                f1, f2, amp, center = vals
                ramp_s = dur if dur is not None else RAMP_DURATION_S
                self._dual.start_temporal_interference_ramp(
                    f1, f2, target_amp_v=amp, center_v=center,
                    duration_s=ramp_s, channel=CHANNEL_A,
                    lut_size=LUT_SIZE)
                self._state.f1 = f1
                self._state.f2 = f2
                self._state.amp = amp
                self._state.center = center
                self._state.running = True

            elif op == "BEAT":
                vals, dur = self._parse_floats_opt_duration(args, 4)
                cfreq, bfreq, amp, cv = vals
                ramp_s = dur if dur is not None else RAMP_DURATION_S
                self._dual.set_beat_frequency_ramp(
                    cfreq, bfreq, target_amp_v=amp, center_v=cv,
                    duration_s=ramp_s, lut_size=LUT_SIZE)
                self._state.f1 = cfreq - bfreq / 2
                self._state.f2 = cfreq + bfreq / 2
                self._state.amp = amp
                self._state.center = cv
                self._state.running = True

            elif op == "UPDATE":
                f1, f2 = self._parse_floats(args, 2)
                if not (self._dual.dac1._waveform_active
                        and self._dual.dac2._waveform_active):
                    self._notify("ERR|UPDATE requires an active waveform")
                    return
                self._dual.dac1.set_frequency(f1)
                self._dual.dac2.set_frequency(f2)
                self._state.f1 = f1
                self._state.f2 = f2
                self._state.running = True

            elif op == "SINE1":
                vals, dur = self._parse_floats_opt_duration(args, 3)
                f, amp, center = vals
                ramp_s = dur if dur is not None else RAMP_DURATION_S
                self._dual.start_sine_ramp(
                    self._dual.dac1, CHANNEL_A, f,
                    target_amp_v=amp, center_v=center,
                    duration_s=ramp_s, lut_size=LUT_SIZE)
                self._state.f1 = f
                self._state.f2 = 0.0
                self._state.amp = amp
                self._state.center = center
                self._state.running = True

            elif op == "SINE2":
                vals, dur = self._parse_floats_opt_duration(args, 3)
                f, amp, center = vals
                ramp_s = dur if dur is not None else RAMP_DURATION_S
                self._dual.start_sine_ramp(
                    self._dual.dac2, CHANNEL_A, f,
                    target_amp_v=amp, center_v=center,
                    duration_s=ramp_s, lut_size=LUT_SIZE)
                self._state.f1 = 0.0
                self._state.f2 = f
                self._state.amp = amp
                self._state.center = center
                self._state.running = True

            elif op == "STOP":
                # Optional override duration as the only argument: STOP|0.5
                if len(args) == 0:
                    ramp_s = RAMP_DURATION_S
                elif len(args) == 1:
                    ramp_s = float(args[0])
                else:
                    raise ValueError("STOP takes 0 or 1 args")
                self._dual.ramp_stop(duration_s=ramp_s)
                self._state.running = False

            elif op == "ESTOP":
                self._dual.emergency_stop()
                self._state.running = False

            elif op == "WAKE":
                # emergency_stop() pulls SHDN low on DAC 1 (which is tied to
                # both DAC SHDN pins on the PCB). wake() releases it.
                self._dual.dac1.wake()

            elif op == "ZERO":
                self._dual._cancel_ramp()
                self._dual.dac1.stop_waveform()
                self._dual.dac2.stop_waveform()
                self._dual.dac1.zero()
                self._dual.dac2.zero()
                self._state.running = False

            else:
                self._notify("ERR|unknown op:{}".format(op))
                return

            self._notify("OK|" + cmd)
            self._notify_state()

        except Exception as e:
            self._notify("ERR|{}".format(e))
            print("cmd error:", e)

    @staticmethod
    def _parse_floats(args, expected):
        if len(args) != expected:
            raise ValueError("expected {} args, got {}".format(
                expected, len(args)))
        return [float(x) for x in args]

    @staticmethod
    def _parse_floats_opt_duration(args, expected):
        """Parse `expected` floats, with an optional trailing duration float.

        Returns (parsed_floats_list, duration_or_None). Allows the BLE
        protocol to grow a per-command ramp duration without breaking
        clients that still send the original argument count.
        """
        n = len(args)
        if n == expected:
            return [float(x) for x in args], None
        if n == expected + 1:
            return [float(x) for x in args[:expected]], float(args[expected])
        raise ValueError("expected {} or {} args, got {}".format(
            expected, expected + 1, n))

    # -- safety ----------------------------------------------------------

    def _safe_stop(self):
        # Disconnects always do an immediate stop. A slow ramp on a lost
        # link is worse than a hard stop, since we cannot guarantee the
        # ramp finishes if the BLE stack is mid-teardown. dual.stop()
        # cancels any in-flight ramp internally.
        try:
            self._dual.stop()
        except Exception as e:
            print("safe_stop failed:", e)
        self._state.running = False


# ----------------------------------------------------------------------
# Entry point
# ----------------------------------------------------------------------

def main():
    print("=" * 40)
    print("PicoW DAC BLE controller")
    print("=" * 40)

    dual = DualDac()
    # Pre-zero so nothing surprises us on the electrodes at boot.
    dual.dac1.zero()
    dual.dac2.zero()

    ble = BLEDacController(dual)
    print("Ready. Waiting for central...")

    try:
        while True:
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\nShutting down...")
        dual.emergency_stop()
        dual.deinit()
        print("Outputs zeroed. Bye.")


if __name__ == "__main__":
    main()
