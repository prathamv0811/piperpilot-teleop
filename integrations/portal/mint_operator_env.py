"""Write a short-lived operator credential file without sharing the API secret."""
import argparse
import datetime
import os
from pathlib import Path

from dotenv import dotenv_values
from livekit import api

from _common import ROOT


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--env-file', type=Path, default=ROOT/'.env')
    ap.add_argument('--output', type=Path, default=ROOT/'.env.operator')
    ap.add_argument('--hours', type=int, default=8)
    args = ap.parse_args()
    if not 1 <= args.hours <= 24:
        raise ValueError('Token lifetime must be between 1 and 24 hours')
    values = dotenv_values(args.env_file)
    room = values.get('LIVEKIT_ROOM', 'piperpilot-dual')
    token = (api.AccessToken(values['LIVEKIT_API_KEY'], values['LIVEKIT_API_SECRET'])
             .with_identity('piperpilot-operator')
             .with_ttl(datetime.timedelta(hours=args.hours))
             .with_grants(api.VideoGrants(room_join=True, room=room, can_publish=True,
                                         can_subscribe=True, can_update_own_metadata=True)).to_jwt())
    fd = os.open(args.output, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
    with os.fdopen(fd, 'w') as f:
        f.write(f"LIVEKIT_URL={values['LIVEKIT_URL']}\nLIVEKIT_ROOM={room}\nLIVEKIT_OPERATOR_TOKEN={token}\n")
    print(f'Wrote {args.output}; valid {args.hours} hours. Transfer privately to the operator host.')


if __name__ == '__main__':
    main()
