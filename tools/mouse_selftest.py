#!/usr/bin/env python3
"""Central self-test orchestrator -- drive the robot's BLE self-test from
the laptop and get a single green/red verdict.

Pairs with hardware/selftest.py running on the board (flashed as code.py
for the verification session).  Connects over BLE, sends one command,
streams the PASS/FAIL results, and exits 0 (all green) or non-zero (red /
timeout) so it slots into a checklist or CI-style gate.

Setup:
    pip install bleak

Usage:
    python3 tools/mouse_selftest.py              # safe sensor checks
    python3 tools/mouse_selftest.py --full       # + motor pulse test (WHEELS MOVE)
    python3 tools/mouse_selftest.py --loops 5     # endurance: 5 reactive loops
    python3 tools/mouse_selftest.py --stop-on-fail
    python3 tools/mouse_selftest.py --name MyMouse --timeout 15

EMERGENCY STOP: press Ctrl-C at any time -- it sends 'stop' to the robot
(motors off) before exiting.  --stop-on-fail also halts on the first FAIL.

Exit codes: 0 green | 1 red/failure | 2 timeout/not-found | 130 Ctrl-C
"""

import argparse
import asyncio
import sys

NUS_RX = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"
NUS_TX = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"


async def orchestrate(args):
    from bleak import BleakScanner, BleakClient

    print("scanning for '{}' ({:.0f}s) ...".format(args.name, args.timeout))
    dev = await BleakScanner.find_device_by_name(args.name, timeout=args.timeout)
    if dev is None:
        print("ERROR: '{}' not found. Powered + advertising? "
              "(selftest.py flashed as code.py?)".format(args.name))
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

    # Pick the command.
    if args.loops:
        command = "loop {}".format(args.loops)
    elif args.full:
        command = "test"
    else:
        command = "check"

    results = []      # (name, ok, detail)
    rc = 2
    try:
        # Give the board a beat to register the connection, then send.
        await asyncio.sleep(0.5)
        print(">>> {}\n".format(command))
        await send(command)

        while True:
            line = await asyncio.wait_for(q.get(), timeout=args.idle_timeout)
            print("   ", line)
            if line.startswith("PASS "):
                p = line.split(" ", 2)
                results.append((p[1], True, p[2] if len(p) > 2 else ""))
            elif line.startswith("FAIL "):
                p = line.split(" ", 2)
                results.append((p[1], False, p[2] if len(p) > 2 else ""))
                if args.stop_on_fail:
                    await send("stop")
                    rc = 1
                    break
            elif line.startswith("GREEN"):
                rc = 0
                break
            elif line.startswith("RED"):
                rc = 1
                break
            elif line.startswith("LOOP done"):
                rc = 0
                break
            elif line.startswith("LOOP aborted") or line == "ABORTED":
                rc = 1
                break
    except asyncio.TimeoutError:
        print("\nTIMEOUT: no response for {:.0f}s.".format(args.idle_timeout))
        rc = 2
    except KeyboardInterrupt:
        print("\n^C -> sending stop")
        rc = 130
    finally:
        try:
            await send("stop")
            await asyncio.sleep(0.2)
            await client.disconnect()
        except Exception:  # noqa: BLE001
            pass

    # Summary.
    if results:
        print("\n----- self-test summary -----")
        npass = 0
        for name, ok, detail in results:
            print("  [{}] {:10s} {}".format("PASS" if ok else "FAIL", name, detail))
            if ok:
                npass += 1
        print("  {}/{} green".format(npass, len(results)))
    verdict = {0: "GREEN -- ready to go", 1: "RED -- fix before running",
               2: "INCOMPLETE", 130: "STOPPED"}.get(rc, "?")
    print("\nVERDICT:", verdict)
    return rc


def main():
    ap = argparse.ArgumentParser(description="Drive the robot BLE self-test.")
    ap.add_argument("--name", default="Micromouse")
    ap.add_argument("--full", action="store_true",
                    help="include the motor pulse test (WHEELS MOVE)")
    ap.add_argument("--loops", type=int, default=0,
                    help="run N endurance reactive-drive loops instead")
    ap.add_argument("--stop-on-fail", action="store_true",
                    help="halt the robot on the first FAIL")
    ap.add_argument("--timeout", type=float, default=10.0,
                    help="BLE scan timeout (s)")
    ap.add_argument("--idle-timeout", type=float, default=30.0,
                    help="max wait between robot messages (s)")
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
