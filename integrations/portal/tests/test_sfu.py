import stat

from dotenv import dotenv_values
import pytest
import yaml

from configure_sfu import prepare


def test_sfu_credentials_are_private_unique_and_no_overwrite(tmp_path):
    first, second = tmp_path/'one', tmp_path/'two'
    prepare(first, '127.0.0.1')
    prepare(second, '100.79.3.30')
    env = dotenv_values(first/'.env.sfu')
    config = yaml.safe_load((first/'livekit.local.yaml').read_text())
    assert config['keys'][env['LIVEKIT_API_KEY']] == env['LIVEKIT_API_SECRET']
    assert len(env['LIVEKIT_API_SECRET']) >= 32
    assert env['LIVEKIT_API_SECRET'] != dotenv_values(second/'.env.sfu')['LIVEKIT_API_SECRET']
    assert stat.S_IMODE(first.stat().st_mode) == 0o700
    assert all(stat.S_IMODE(f.stat().st_mode) == 0o600 for f in first.iterdir())
    assert config['bind_addresses'] == ['127.0.0.1']
    assert config['rtc']['tcp_port'] == 0
    assert config['rtc']['ips']['includes'] == ['127.0.0.1/32']
    with pytest.raises(FileExistsError):
        prepare(first, '127.0.0.1')
    assert dotenv_values(first/'.env.sfu') == env


@pytest.mark.parametrize('address', ['0.0.0.0', '8.8.8.8', '::1'])
def test_rejects_unspecified_and_public_binding(tmp_path, address):
    with pytest.raises(ValueError):
        prepare(tmp_path/'unused', address)
    assert not (tmp_path/'unused').exists()
