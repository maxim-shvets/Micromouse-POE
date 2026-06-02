"""BLE remote control for the micromouse (Nordic UART Service).

The XIAO nRF52840 Sense has a BLE radio, so we don't need a physical
start button -- a phone runs the show over a BLE UART link.

Pair with any BLE UART terminal app:
  - "Bluefruit Connect" (Adafruit, iOS/Android) -> UART tab
  - "Serial Bluetooth Terminal" (Android)
  - "nRF Toolbox" -> UART
Connect to the device advertised as "Micromouse", then send text
commands terminated by a newline (most apps have a newline toggle).

Commands (case-insensitive, newline-terminated):
    go / run     run the full flow (explore -> return -> speed)
    explore      explore to the centre only
    return       drive back to the start (uses the known map)
    speed        speed run to the centre (uses the known map)
    stop / x     EMERGENCY STOP -- abort the current phase, motors off
    reset        forget the mapped maze (start exploration fresh)
    status / ?   report the current state

Status lines are sent back over the same link so the phone shows
progress ("EXPLORE...", "reached (7, 7)", "DONE", "ABORT ...").

Defensive: if the `adafruit_ble` library isn't present, `available` is
False and `code.py` falls back to the countdown start.  Lazy imports keep
this file importable on CPython for syntax / lint.

CircuitPython-portable: no f-strings, plain class.
"""


class BleControl(object):
    """Nordic UART Service wrapper with non-blocking command polling."""

    def __init__(self, name="Micromouse"):
        self.available = False
        self._buf = ""
        self._advertising = False
        self._radio = None
        self._uart = None
        self._adv = None
        try:
            import adafruit_ble
            from adafruit_ble.advertising.standard import (
                ProvideServicesAdvertisement)
            from adafruit_ble.services.nordic import UARTService
            self._radio = adafruit_ble.BLERadio()
            self._uart = UARTService()
            self._adv = ProvideServicesAdvertisement(self._uart)
            try:
                self._radio.name = name
            except Exception:  # noqa: BLE001
                pass
            self.available = True
        except Exception:  # noqa: BLE001
            self.available = False

    # ---- connection lifecycle ------------------------------------------

    @property
    def connected(self):
        try:
            return bool(self.available and self._radio.connected)
        except Exception:  # noqa: BLE001
            return False

    def service(self):
        """Keep advertising while disconnected.  Call periodically."""
        if not self.available:
            return
        try:
            if self._radio.connected:
                self._advertising = False
            elif not self._advertising:
                self._radio.start_advertising(self._adv)
                self._advertising = True
        except Exception:  # noqa: BLE001
            pass

    # ---- I/O ------------------------------------------------------------

    def poll(self):
        """Return a list of complete command tokens received since last call.

        Non-blocking: reads only what's already buffered.  Tokens are
        lower-cased and stripped; commands must be newline/CR terminated.
        """
        cmds = []
        if not self.connected:
            return cmds
        try:
            n = self._uart.in_waiting
            if not n:
                return cmds
            data = self._uart.read(n)
            if not data:
                return cmds
            for byte in data:
                ch = chr(byte)
                if ch == "\n" or ch == "\r":
                    tok = self._buf.strip().lower()
                    if tok:
                        cmds.append(tok)
                    self._buf = ""
                else:
                    self._buf += ch
                    if len(self._buf) > 64:      # runaway guard
                        self._buf = self._buf[-64:]
        except Exception:  # noqa: BLE001
            pass
        return cmds

    def send(self, msg):
        """Send a status line to the phone (no-op if disconnected)."""
        if not self.connected:
            return
        try:
            self._uart.write((msg + "\n").encode("utf-8"))
        except Exception:  # noqa: BLE001
            pass
