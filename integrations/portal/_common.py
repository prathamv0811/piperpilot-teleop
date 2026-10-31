"""Connection configuration; credentials never enter the wire schema or logs."""
import datetime
import json
import os
from pathlib import Path

ROOT = Path(__file__).resolve().parent


def connection(identity, role, env_file=None):
    from dotenv import load_dotenv
    load_dotenv(env_file or ROOT/'.env', override=False)
    url = os.environ.get('LIVEKIT_URL')
    room = os.environ.get('LIVEKIT_ROOM', 'piperpilot-dual')
    if not url:
        raise ValueError('Set LIVEKIT_URL in the environment or --env-file')
    token = os.environ.get(f'LIVEKIT_{role.upper()}_TOKEN')
    if not token:
        from livekit import api
        key, secret = os.environ.get('LIVEKIT_API_KEY'), os.environ.get('LIVEKIT_API_SECRET')
        if not key or not secret:
            raise ValueError(f'Set LIVEKIT_{role.upper()}_TOKEN or the LiveKit API key/secret')
        token = (api.AccessToken(key, secret).with_identity(identity)
                 .with_ttl(datetime.timedelta(hours=2))
                 .with_grants(api.VideoGrants(room_join=True, room=room, can_publish=True,
                                             can_subscribe=True, can_update_own_metadata=True)).to_jwt())
    return url, room, token


def save_report(path, value):
    if path:
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        temp = path.with_suffix('.tmp')
        temp.write_text(json.dumps(value, indent=2)+'\n')
        temp.replace(path)
