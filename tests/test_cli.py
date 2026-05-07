from ailamp.cli import main


def test_cli_static_hardware_check_passes(capsys):
    exit_code = main(["hardware-check"])
    output = capsys.readouterr().out

    assert exit_code == 0
    assert "PASS controller.model: NVIDIA Jetson Orin Nano Super Developer Kit" in output
    assert "PASS controller.mpn: 945-13766-0000-000" in output
    assert "PASS led.count: 64" in output
    assert "PASS bom.servo.quantity: 5" in output


def test_cli_hardware_check_failures_only_is_empty(capsys):
    exit_code = main(["hardware-check", "--failures-only"])
    output = capsys.readouterr().out

    assert exit_code == 0
    assert output == ""


def test_cli_birthday_check_dry_run(capsys):
    exit_code = main(["birthday-check", "--today", "2026-05-08", "--dry-run"])
    output = capsys.readouterr().out

    assert exit_code == 0
    assert "is_birthday=True" in output
    assert "should_play=True" in output
    assert "Happy birthday, Yugu!" in output


def test_cli_sim_demo_outputs_virtual_vision_events(capsys):
    exit_code = main(["sim-demo"])
    output = capsys.readouterr().out

    assert exit_code == 0
    assert "event=no_person motion=scanning" in output
    assert "event=person_left motion=headshake" in output
    assert "event=person_close motion=shy" in output


def test_cli_vision_loop_uses_runtime(monkeypatch, capsys):
    calls = {}

    class FakeResult:
        def format(self):
            return "frame=0 event=person_center motion=nod applied=True"

    class FakeRuntime:
        def __init__(self, config):
            calls["project"] = config.system.project_name

        def open(self, *, with_outputs):
            calls["open_with_outputs"] = with_outputs

        def run(self, *, max_frames, interval_s, apply_outputs):
            calls["run"] = (max_frames, interval_s, apply_outputs)
            yield FakeResult()

        def close(self):
            calls["closed"] = True

    monkeypatch.setattr("ailamp.cli.VisionRuntime", FakeRuntime)

    exit_code = main(["vision-loop", "--frames", "2", "--interval", "0", "--with-outputs"])
    output = capsys.readouterr().out

    assert exit_code == 0
    assert calls == {
        "project": "AILamp",
        "open_with_outputs": True,
        "run": (2, 0.0, True),
        "closed": True,
    }
    assert "event=person_center" in output
