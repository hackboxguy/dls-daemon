#!/usr/bin/env python3
"""
Test client for dls-daemon

Connects to the data socket and displays received packets.
"""

import argparse
import json
import socket
import sys
import os

def connect_unix(path):
    """Connect to Unix domain socket"""
    sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    sock.connect(path)
    return sock

def connect_tcp(host, port):
    """Connect to TCP socket"""
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((host, port))
    return sock

def send_json(sock, data):
    """Send JSON message"""
    msg = json.dumps(data) + '\n'
    sock.sendall(msg.encode())

def recv_json(sock):
    """Receive JSON message (newline-delimited)"""
    buf = b''
    while True:
        c = sock.recv(1)
        if not c:
            return None
        if c == b'\n':
            break
        buf += c
    return json.loads(buf.decode())

def main():
    parser = argparse.ArgumentParser(description='DLS daemon test client')
    parser.add_argument('--unix', '-u', default='/tmp/dls-data.sock',
                       help='Unix socket path')
    parser.add_argument('--tcp', '-t', metavar='HOST:PORT',
                       help='TCP host:port (instead of Unix socket)')
    parser.add_argument('--categories', '-c', nargs='+',
                       help='Filter by categories (e.g., VERSION BOOT)')
    parser.add_argument('--msg-ids', '-m', nargs='+', type=int,
                       help='Filter by message IDs')
    parser.add_argument('--raw', '-r', action='store_true',
                       help='Show raw JSON output')
    parser.add_argument('--count', '-n', type=int, default=0,
                       help='Stop after N packets (0 = unlimited)')
    args = parser.parse_args()

    # Connect
    if args.tcp:
        host, port = args.tcp.split(':')
        sock = connect_tcp(host, int(port))
        print(f"Connected to TCP {host}:{port}")
    else:
        sock = connect_unix(args.unix)
        print(f"Connected to Unix socket {args.unix}")

    # Receive welcome
    welcome = recv_json(sock)
    if welcome:
        print(f"Welcome: client_id={welcome.get('client_id')}, version={welcome.get('version')}")

    # Subscribe
    subscribe = {'cmd': 'subscribe'}
    if args.categories:
        subscribe['categories'] = args.categories
    if args.msg_ids:
        subscribe['msg_ids'] = args.msg_ids
    send_json(sock, subscribe)

    # Wait for confirmation
    resp = recv_json(sock)
    if resp and resp.get('type') == 'subscribed':
        print("Subscribed successfully")
    else:
        print(f"Subscribe response: {resp}")

    print("\nReceiving packets (Ctrl+C to stop)...\n")

    # Receive packets
    count = 0
    try:
        while True:
            pkt = recv_json(sock)
            if not pkt:
                print("\nConnection closed")
                break

            if pkt.get('type') != 'packet':
                continue

            count += 1

            if args.raw:
                print(json.dumps(pkt))
            else:
                # Format packet
                ts = pkt.get('ts', 0)
                seq = pkt.get('seq', 0)
                msg_id = pkt.get('msg_id_hex', '0x????????')
                category = pkt.get('category', '')
                name = pkt.get('name', '')
                payload_hex = pkt.get('payload_hex', '')
                payload_ascii = pkt.get('payload_ascii', '')

                line = f"[{seq:6d}] {msg_id}"
                if category:
                    line += f" [{category}]"
                if name:
                    line += f" {name}"
                if payload_hex:
                    line += f" | {payload_hex[:40]}"
                    if len(payload_hex) > 40:
                        line += "..."
                if payload_ascii:
                    line += f" | '{payload_ascii[:20]}'"
                    if len(payload_ascii) > 20:
                        line += "..."

                print(line)

            if args.count > 0 and count >= args.count:
                print(f"\nReceived {count} packets")
                break

    except KeyboardInterrupt:
        print(f"\n\nInterrupted. Received {count} packets.")

    sock.close()

if __name__ == '__main__':
    main()
