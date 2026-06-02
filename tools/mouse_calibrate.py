#!/usr/bin/env python3
"""Drive the robot's automatic calibration from the laptop and capture the
result.

Pairs with hardware/calibrate.py running on the board (flashed as code.py
for the calibration session).  Sends the auto-calibration sequence, streams
the progress, collects every "SET <key>=<value>" the robot reports, and
writes them to a local tunables.json so you have the profile on the laptop
too.  Optionally tells the board to persist it as well.

Setup:
    pip install bleak

Usage:
    python3 tools/mouse_calibrate.py                 # run auto-cal, save local JSON
    python3 tools/mouse_calibrate.py --out my.json
    python3 tools/mouse_calibrate.py --save          # also persist on the board
    python3 tools/mouse_calibrate.py --cmd "cal gyro"   # just one routine

The motion routines (wheelbase / encoder) make the WHEELS MOVE -- the robot
warns and counts down first; press Ctrl-C to abort (sends stop).

Exit codes: 0 ok | 2 timeout/not-found | 130 Ctrl-C
"""

import argparse
import asyncio
import json
import sys

NUS_RX = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"
NUS_TX = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"


def _coerce(v):
    """Turn a reported value string into int / float / bool / str."""
    low = v.lower()
    if low in ("true", "false"):
        return low == "true"
    try:
        return int(v)
    except ValueError:
        pass
    try:
        return float(v)
    except ValueError:
        return v


async def orchestrate(args):
    from bleak import BleakScanner, BleakClient

    print("scanning for '{}' ({:.0f}s) ...".format(args.name, args.timeout))
    dev = await BleakScanner.find_device_by_name(args.name, timeout=args.timeout)
    if dev is None:
        print("ERROR: '{}' not found. Powered + advertising? "
              "(calibrate.py flashed as code.py?)".format(args.name))
        return 2

    q = asyncio.Queue()
    buf = {"s": ""}

    def on_notify(_char, data):
        buf["s"] += bytes(data).decode("utf-8", "replace")
        while "\n" in buf["s"]:
            line, buf["s"] = buf["s"].split("\n", 1)
            line = line.strip("\r ")
            if line:
                q.put_nowait(line)

    client = BleakClient(dev)
    await client.connect()
    await client.start_notify(NUS_TX, on_notify)
    print("connected.\n")

    async def send(cmd):
        await client.write_gatt_char(NUS_RX, (cmd + "\n").encode("utf-8"),
                                     response=False)

    captured = {}
    rc = 2
    try:
        await asyncio.sleep(0.5)
        print(">>> {}\n".format(args.cmd))
        await send(args.cmd)

        while True:
            line = await asyncio.wait_for(q.get(), timeout=args.idle_timeout)
            print("   ", line)
            if line.startswith("SET "):
                kv = line[4:]
                if "=" in kv:
                    k, _, v = kv.partition("=")
                    captured[k.strip()] = _coerce(v.strip())
            elif line.startswith("CAL done"):
                rc = 0
                break
            elif line.startswith("CAL ") and args.cmd != "cal":
                # single routine: its CAL summary line is the terminal.
                rc = 0
                break
    except asyncio.TimeoutError:
        # For single routines without a 'done' marker, idle just means finished.
        rc = 0 if captured else 2
        if rc == 2:
            print("\nTIMEOUT: no response.")
    except KeyboardInterrupt:
        print("\n^C -> sending stop")
        rc = 130
    finally:
        try:
            await send("stop")
            await asyncio.sleep(0.2)
            if args.save and captured and rc == 0:
                print("\npersisting on the board ...")
                await send("save")
                await asyncio.sleep(0.5)
            await client.disconnect()
        except Exception:  # noqa: BLE001
            pass

    if captured:
        print("\n----- calibrated values -----")
        for k in sorted(captured):
            print("  {} = {}".format(k, captured[k]))
        with open(args.out, "w") as f:
            json.dump(captured, f, indent=2)
            f.write("\n")
        print("\nwrote {} ({} keys)".format(args.out, len(captured)))
        print("copy it to the board as /tunables.json (or re-run with --save).")
    else:
        print("\n(no calibration values captured)")
    return rc


def main():
    ap = argparse.ArgumentParser(description="Drive the robot auto-calibration.")
    ap.add_argument("--name", default="Micromouse")
    ap.add_argument("--cmd", default="cal",
                    help="calibration command to run (default: cal). "
                         "e.g. 'cal gyro', 'cal wheelbase', 'cal encoder'")
    ap.add_argument("--out", default="tunables.json",
                    help="local JSON file to write (default: tunables.json)")
    ap.add_argument("--save", action="store_true",
                    help="also tell the board to persist /tunables.json")
    ap.add_argument("--timeout", type=float, default=10.0)
    ap.add_argument("--idle-timeout", type=float, default=30.0)
    args = ap.parse_args()

    try:
        import bleak  # noqa: F401
    except ImportError:
        print("This tool needs the 'bleak' BLE library:\n    pip install bleak")
        return 2

    try:
        return asyncio.run(orchestrate(args))
    except KeyboardInterrupt:
        return 130


if __name__ == "__main__":
    sys.exit(main())
