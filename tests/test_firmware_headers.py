"""The nose-lift module keeps its decision logic in headers independent of PX4 (PilotGate.hpp, RcTrigger.hpp, ...), each
with a plain C++ test in tests/firmware. They used to be compiled by hand; this runs every one of them with the suite."""
import functools
import glob
import os
import shutil
import subprocess
from pathlib import Path

import pytest

TESTS = sorted((Path(__file__).parent / "firmware").glob("test_*.cpp"))
CXX = shutil.which("c++") or shutil.which("clang++") or shutil.which("g++")


def _build(src: Path, exe: Path, env: dict) -> subprocess.CompletedProcess:
    return subprocess.run([CXX, "-std=c++17", "-O1", str(src), "-o", str(exe)], capture_output=True, text=True, env=env)


@functools.lru_cache(maxsize=1)
def _env() -> dict:
    """The compiler's default environment, or - on a Mac whose newest SDK the installed linker cannot read (the same
    thing the PX4 build trips over) - the newest older SDK that links."""
    env = dict(os.environ)
    if env.get("SDKROOT") or not os.path.isdir("/Library/Developer/CommandLineTools/SDKs"):
        return env
    probe = Path(__file__).parent / "firmware" / "test_rc_trigger.cpp"
    out = Path(os.environ.get("TMPDIR", "/tmp")) / "afd_sdk_probe"
    if _build(probe, out, env).returncode == 0:
        return env
    for sdk in sorted(glob.glob("/Library/Developer/CommandLineTools/SDKs/MacOSX[0-9]*.*.sdk"), reverse=True):
        trial = {**env, "SDKROOT": sdk}
        if _build(probe, out, trial).returncode == 0:
            return trial
    return env


@pytest.mark.skipif(CXX is None, reason="no C++ compiler")
@pytest.mark.parametrize("src", TESTS, ids=[p.stem for p in TESTS])
def test_firmware_header(src, tmp_path):
    if "#include <PositionControl.hpp>" in src.read_text():
        pytest.skip("builds against PX4's PositionControl: needs the PX4 tree, not just the module's headers")
    exe = tmp_path / src.stem
    built = _build(src, exe, _env())
    assert built.returncode == 0, built.stderr[-2000:]
    ran = subprocess.run([str(exe)], capture_output=True, text=True, timeout=30)
    assert ran.returncode == 0, (ran.stdout + ran.stderr)[-2000:]
