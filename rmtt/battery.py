#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import annotations

import argparse
import multiprocessing as mp
import queue
import socket
import sys
import time

from rmtt.adapter import RMTTClient
from rmtt_config import DEFAULT_RMTT_IP


DEFAULT_DRONE_IP = DEFAULT_RMTT_IP
DEFAULT_DRONE_CHECK_TIMEOUT_SEC = 8.0
TELLO_COMMAND_PORT = 8889


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Print RMTT battery percentage")
    parser.add_argument("--ip", default=DEFAULT_DRONE_IP, help="RMTT IP address")
    parser.add_argument("--interval", type=float, default=1.0, help="print interval seconds")
    parser.add_argument("--timeout", type=float, default=DEFAULT_DRONE_CHECK_TIMEOUT_SEC)
    parser.add_argument("--repeat", action="store_true", help="keep polling until Ctrl+C")
    args = parser.parse_args(argv)

    try:
        while True:
            status, payload = read_drone_battery_isolated(args.ip, timeout=args.timeout)
            timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
            if status == "ok":
                print("[{0}] Battery: {1}%".format(timestamp, payload), flush=True)
                rc = 0
            elif status == "timeout":
                print("[{0}] Battery read timed out after {1:.1f}s".format(timestamp, args.timeout), flush=True)
                rc = 1
            else:
                print("[{0}] Battery read failed: {1}".format(timestamp, payload), flush=True)
                rc = 1
            if not args.repeat:
                return rc
            time.sleep(max(0.1, args.interval))
    except KeyboardInterrupt:
        print("\nStopped", flush=True)
        return 130


def read_drone_battery_isolated(
    ip: str,
    *,
    timeout: float = DEFAULT_DRONE_CHECK_TIMEOUT_SEC,
) -> tuple[str, int | str | None]:
    udp_status, udp_payload = read_tello_battery_udp(ip, timeout=min(float(timeout), 2.0))
    if udp_status == "ok":
        return udp_status, udp_payload
    queue_out: mp.Queue = mp.Queue()
    process = mp.Process(target=_drone_check_worker, args=(ip, queue_out))
    process.start()
    try:
        status, payload = queue_out.get(timeout=max(0.1, float(timeout)))
    except queue.Empty:
        process.terminate()
        process.join(1.0)
        return "timeout", None
    if process.is_alive():
        process.terminate()
    process.join(1.0)
    return status, payload


def read_tello_battery_udp(ip: str, *, timeout: float = 2.0) -> tuple[str, int | str | None]:
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.settimeout(max(0.1, float(timeout)))
            sock.bind(("", 0))
            address = (ip, TELLO_COMMAND_PORT)
            sock.sendto(b"command", address)
            response, _ = sock.recvfrom(1024)
            if response.strip().lower() != b"ok":
                return "error", "SDK command rejected: {0!r}".format(response)
            sock.sendto(b"battery?", address)
            response, _ = sock.recvfrom(1024)
    except socket.timeout:
        return "timeout", None
    except OSError as exc:
        return "error", str(exc)
    text = response.decode("utf-8", errors="replace").strip()
    try:
        return "ok", int(text)
    except ValueError:
        return "error", "unexpected battery response: {0!r}".format(text)


def _drone_check_worker(ip: str, queue_out: mp.Queue) -> None:
    client = RMTTClient(ip)
    try:
        client.connect()
        battery = client.battery_percent()
        queue_out.put(("ok", battery))
    except Exception as exc:
        queue_out.put(("error", str(exc)))
    finally:
        client.close()


if __name__ == "__main__":
    sys.exit(main())
