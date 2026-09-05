"""Exercise actual node source with a fake UART; no ESP32 SDK, device or power is used."""
import shutil
import subprocess
from pathlib import Path

import pytest


def test_node_rejects_bad_packets_and_never_enables_parking(tmp_path):
    compiler = shutil.which('clang++') or shutil.which('g++')
    if compiler is None:
        pytest.skip('C++ compiler unavailable')
    native = Path(__file__).parent / 'native'
    executable = tmp_path / 'node-test'
    subprocess.run([compiler, '-std=c++17', '-fsanitize=address,undefined', '-g',
                    '-I', str(native), str(native / 'node_test.cpp'), '-o', str(executable)], check=True)
    subprocess.run([str(executable)], check=True)
