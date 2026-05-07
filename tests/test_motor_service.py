import builtins

import pytest

from ailamp.services.motor import MotorService


def test_motor_service_reports_missing_upstream_runtime(monkeypatch):
    real_import = builtins.__import__

    def fake_import(name, globals=None, locals=None, fromlist=(), level=0):
        if name == "lelamp.service.motors.animation_service":
            raise ModuleNotFoundError(name)
        return real_import(name, globals, locals, fromlist, level)

    monkeypatch.setattr(builtins, "__import__", fake_import)
    service = MotorService("/dev/null", "ailamp", "ailamp_runtime/ailamp/recordings")

    with pytest.raises(RuntimeError, match="upstream LeLamp runtime"):
        service.connect()
