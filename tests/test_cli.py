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


def test_cli_sim_demo_outputs_virtual_vision_events(capsys):
    exit_code = main(["sim-demo"])
    output = capsys.readouterr().out

    assert exit_code == 0
    assert "event=no_person motion=scanning" in output
    assert "event=person_left motion=headshake" in output
    assert "event=person_close motion=shy" in output
