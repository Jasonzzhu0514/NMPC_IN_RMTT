#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import annotations

import argparse
import logging
import socket
import sys

import netaddr
import netifaces
from multi_robomaster import tool


def safe_get_subnets():
    subnets = []
    addr_list = []

    for iface in netifaces.interfaces():
        addrs = netifaces.ifaddresses(iface)
        if socket.AF_INET not in addrs:
            continue

        ipinfo = addrs[socket.AF_INET][0]
        address = ipinfo.get("addr")
        netmask = ipinfo.get("netmask")

        if not address or not netmask:
            continue
        if address.startswith("127."):
            continue
        if netmask != "255.255.255.0":
            continue

        cidr = netaddr.IPNetwork("%s/%s" % (address, netmask))
        subnets.append((cidr.network, netmask))
        addr_list.append(address)

    return subnets, addr_list


def scan_ip(num):
    tool.get_subnets = safe_get_subnets

    logger = logging.getLogger("multi_robot")
    logger.setLevel(logging.DEBUG)

    client = tool.TelloClient()
    client._conn.local_ip = "0.0.0.0"
    client.start()

    try:
        robot_host_list = client.scan_multi_robot(num)
        for host in robot_host_list:
            print("scan result: host{0}".format(host))
    finally:
        client.close()


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("-n", "--num", type=int, default=1)
    args = parser.parse_args(argv)

    print("Scan IP ...")
    scan_ip(args.num)
    return 0


if __name__ == "__main__":
    sys.exit(main())
