"""Prepare a local/private-network LiveKit SFU; never starts it or changes firewall rules."""
import argparse
import ipaddress
import os
from pathlib import Path
import secrets

import yaml

from _common import ROOT


def prepare(directory, address, port=18780, room='piperpilot-dual'):
    address = ipaddress.IPv4Address(address)
    allowed = ('127.0.0.0/8', '10.0.0.0/8', '172.16.0.0/12', '192.168.0.0/16', '100.64.0.0/10')
    if not any(address in ipaddress.ip_network(net) for net in allowed):
        raise ValueError('Use a loopback, LAN, or Tailscale IPv4 address; public hosting requires TLS setup')
    if not 1024 <= port <= 65533:
        raise ValueError('Base port must be between 1024 and 65533')
    if not room or any(c not in 'abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789-_' for c in room):
        raise ValueError('Room must contain only letters, numbers, underscores, or hyphens')
    address = str(address)
    key, secret = 'piper_'+secrets.token_hex(12), secrets.token_urlsafe(48)
    config = {'port': port, 'bind_addresses': [address], 'development': False,
              'rtc': {'node_ip': address, 'use_external_ip': False, 'tcp_port': 0,
                      'udp_port': port+2, 'enable_loopback_candidate': address.startswith('127.'),
                      'ips': {'includes': [address+'/32']}},
              'keys': {key: secret}, 'logging': {'level': 'warn'}}
    directory = Path(directory)
    directory.mkdir(mode=0o700)
    files = {'livekit.local.yaml': yaml.safe_dump(config, sort_keys=False),
             '.env.sfu': f'LIVEKIT_URL=ws://{address}:{port}\nLIVEKIT_ROOM={room}\n'
                         f'LIVEKIT_API_KEY={key}\nLIVEKIT_API_SECRET={secret}\n'}
    for name, content in files.items():
        fd = os.open(directory/name, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
        with os.fdopen(fd, 'w') as f:
            f.write(content)
    return directory


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--bind-address', default='127.0.0.1')
    ap.add_argument('--port', type=int, default=18780)
    ap.add_argument('--room', default='piperpilot-dual')
    ap.add_argument('--output-dir', type=Path, default=ROOT/'.sfu')
    args = ap.parse_args()
    directory = prepare(args.output_dir, args.bind_address, args.port, args.room)
    print(f'Prepared {directory}; no server started. Keep this directory on the robot host.')


if __name__ == '__main__':
    main()
