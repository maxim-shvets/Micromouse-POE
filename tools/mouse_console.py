#!/usr/bin/env python3
"""Mac/PC BLE console for the micromouse -- no phone app needed.

Connects to the robot's Nordic UART Service (the same one
hardware/ble_control.py exposes) over Bluetooth LE and gives you an
interactive command line.  Type a command, press Enter; status lines from
the robot print as they arrive.

Setup (once):
    pip install bleak

macOS note: the first run pops a Bluetooth permission prompt for your
terminal app -- allow it (System Settings -> Privacy & Security ->
Bluetooth).  Core Bluetooth hides MAC addresses, so we connect by the
advertised name ("Micromouse").

Usage:
    python3 tools/mouse_console.py                 # find + connect "Micromouse"
    python3 tools/mouse_console.py --name MyMouse
    python3 tools/mouse_console.py --scan          # list nearby BLE devices
    python3 tools/mouse_console.py --address <uuid>  # connect directly (macOS UUID)
    python3 tools/mouse_console.py --log run.txt   # tee status to a file

Robot commands (sent verbatim, newline-appended):
    go / run     full flow: explore -> return -> speed
    explore | return | speed     one phase
    stop         EMERGENCY STOP (also sent automatically on Ctrl-C / quit)
    reset        forget the mapped maze
    status       report current cell
Local commands:
    help         show this list
    scan         re-scan (only when disconnected)
    quit / q     stop the robot, disconnect, exit
"""

import argparse
import asyncio
import queue
import sys
import threading

# Nordic UART Service UUIDs (must match adafruit_ble.services.nordic.UARTService).
NUS_SERVICE = "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
NUS_RX = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"   # central -> peripheral (write)
NUS_TX = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"   # peripheral -> central (notify)

BANNER = """\
------------------------------------------------------------
  Micromouse BLE console
  robot : go  explore  return  speed  stop  reset  status
  local : help  quit
  (Ctrl-C sends 'stop' then exits)
------------------------------------------------------------"""

_SENTINEL = object()


def _start_stdin_reader():
    """Background daemon thread feeding stdin lines into a queue.

    Daemon so a blocked readline never prevents process exit.
    """
    q = queue.Queue()

    def reader():
        try:
            for line in sys.stdin:
                q.put(line)
        except Exception:  # noqa: BLE001
            pass
        q.put(None)

    threading.Thread(target=reader, daemon=True).start()
    return q


def _q_get(q):
    try:
        return q.get(timeout=0.2)
    except queue.Empty:
        return _SENTINEL


async def find_device(name, address, timeout):
    from bleak import BleakScanner
    if address:
        return address
    print("scanning for '{}' ({:.0f}s) ...".format(name, timeout))
    dev = await BleakScanner.find_device_by_name(name, timeout=timeout)
    return dev


async def do_scan(timeout):
    from bleak import BleakScanner
    print("scanning {:.0f}s ...".format(timeout))
    devices = await BleakScanner.discover(timeout=timeout)
    if not devices:
        print("  (no BLE devices found)")
        return
    for d in devices:
        print("  {:40s}  {}".format(d.address, d.name or "(unnamed)"))


async def run_console(device, log_path):
    from bleak import BleakClient

    logf = open(log_path, "a") if log_path else None

    def on_notify(_char, data):
        # The robot sends newline-terminated status; chunks may split lines.
        on_notify.buf += bytes(data).decode("utf-8", "replace")
        while "\n" in on_notify.buf:
            line, on_notify.buf = on_notify.buf.split("\n", 1)
            line = line.rstrip("\r")
            if line:
                sys.stdout.write("\r<robot> {}\n> ".format(line))
                sys.stdout.flush()
                if logf:
                    logf.write(line + "\n")
                    logf.flush()
    on_notify.buf = ""

    disconnected = asyncio.Event()

    def on_disconnect(_client):
        disconnected.set()

    client = BleakClient(device, disconnected_callback=on_disconnect)
    await client.connect()
    print("connected.")
    await client.start_notify(NUS_TX, on_notify)
    print(BANNER)

    q = _start_stdin_reader()
    loop = asyncio.get_event_loop()

    async def send(cmd):
        await client.write_gatt_char(NUS_RX, (cmd + "\n").encode("utf-8"),
                                     response=False)

    try:
        sys.stdout.write("> ")
        sys.stdout.flush()
        while client.is_connected and not disconnected.is_set():
            line = await loop.run_in_executor(None, _q_get, q)
            if line is _SENTINEL:
                continue                      # timeout tick -> re-check links
            if line is None:
                break                         # stdin EOF
            cmd = line.strip()
            if not cmd:
                sys.stdout.write("> ")
                sys.stdout.flush()
                continue
            low = cmd.lower()
            if low in ("quit", "q", "exit"):
                break
            if low == "help":
                print(BANNER)
            elif low == "scan":
                print("(already connected; quit first to scan)")
            else:
                await send(cmd)
            sys.stdout.write("> ")
            sys.stdout.flush()
    except KeyboardInterrupt:
        print("\n^C")
    finally:
        # Safety: always try to stop the motors before leaving.
        try:
            if client.is_connected:
                await send("stop")
                await asyncio.sleep(0.2)
                await client.disconnect()
        except Exception:  # noqa: BLE001
            pass
        if logf:
            logf.close()
        print("disconnected.")


async def amain(args):
    if args.scan:
        await do_scan(args.timeout)
        return 0
    device = await find_device(args.name, args.address, args.timeout)
    if device is None:
        print("ERROR: '{}' not found. Is the robot powered + advertising? "
              "Try --scan.".format(args.name))
        return 1
    await run_console(device, args.log)
    return 0


def main():
    ap = argparse.ArgumentParser(description="BLE console for the micromouse.")
    ap.add_argument("--name", default="Micromouse",
                    help="advertised device name (default: Micromouse)")
    ap.add_argument("--address", default=None,
                    help="connect directly by address/UUID (skip scan)")
    ap.add_argument("--scan", action="store_true",
                    help="list nearby BLE devices and exit")
    ap.add_argument("--timeout", type=float, default=10.0,
                    help="scan timeout seconds (default 10)")
    ap.add_argument("--log", default=None,
                    help="append robot status lines to this file")
    args = ap.parse_args()

    try:
        import bleak  # noqa: F401
    except ImportError:
        print("This tool needs the 'bleak' BLE library:\n    pip install bleak")
        return 2

    try:
        return asyncio.run(amain(args))
    except KeyboardInterrupt:
        return 0


if __name__ == "__main__":
    sys.exit(main())
