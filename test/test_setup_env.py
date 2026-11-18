"""scripts/setup_env.sh 가 환경별 설정 파일을 만드는지 확인한다(HOME 대신 임시 폴더 사용)."""
import os
import subprocess
from pathlib import Path

import pytest
import yaml

SCRIPT = Path(__file__).resolve().parents[1] / 'scripts' / 'setup_env.sh'


def run(tmp_path, *args):
    env = {**os.environ, 'HOME': str(tmp_path), 'PINKY_SIM_CONFIG_DIR': str(tmp_path / 'cfg')}
    env.pop('ROS_DOMAIN_ID', None)
    r = subprocess.run(['bash', str(SCRIPT), *args], env=env, capture_output=True, text=True)
    return r, tmp_path / 'cfg'


@pytest.mark.parametrize('profile, stop', [('wsl', 0.20), ('native', 0.20)])
def test_profile_values(tmp_path, profile, stop):
    r, cfg = run(tmp_path, '--profile', profile, '--domain', '7')
    assert r.returncode == 0, r.stdout + r.stderr
    params = yaml.safe_load((cfg / 'runtime.yaml').read_text(encoding='utf-8'))
    assert params['command_driver']['ros__parameters']['stop_distance'] == stop
    env_sh = (cfg / 'env.sh').read_text(encoding='utf-8')
    assert f'PINKY_SIM_PROFILE={profile}' in env_sh and 'ROS_DOMAIN_ID=7' in env_sh
    assert 'LIBGL_ALWAYS_SOFTWARE' not in env_sh


def test_software_gl_and_bashrc_once(tmp_path):
    run(tmp_path, '--software-gl', '--bashrc')
    run(tmp_path, '--software-gl', '--bashrc')
    assert 'LIBGL_ALWAYS_SOFTWARE=1' in (tmp_path / 'cfg' / 'env.sh').read_text()
    assert (tmp_path / '.bashrc').read_text().count('pinky_sim_examples') == 1


@pytest.mark.parametrize('args', [['--profile', 'mac'], ['--domain', 'abc'], ['--domain', '300'],
                                  ['--nope']])
def test_bad_arguments(tmp_path, args):
    r, cfg = run(tmp_path, *args)
    assert r.returncode == 2 and not (cfg / 'runtime.yaml').exists()


def test_windows_fonts_links_malgun(tmp_path):
    win = tmp_path / 'winfonts'
    win.mkdir()
    (win / 'malgun.ttf').write_bytes(b'fake')
    env = {**os.environ, 'HOME': str(tmp_path), 'PINKY_SIM_CONFIG_DIR': str(tmp_path / 'cfg'),
           'PINKY_WIN_FONT_DIR': str(win)}
    r = subprocess.run(['bash', str(SCRIPT), '--profile', 'wsl', '--windows-fonts'],
                       env=env, capture_output=True, text=True)
    assert r.returncode == 0
    link = tmp_path / '.local' / 'share' / 'fonts' / 'malgun.ttf'
    assert link.is_symlink() and link.resolve() == (win / 'malgun.ttf').resolve()
