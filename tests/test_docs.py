from pathlib import Path


def test_docs_do_not_reference_missing_motion_names():
    root = Path(__file__).resolve().parents[1]
    docs_text = "\n".join(path.read_text() for path in (root / "docs").rglob("*.md"))

    assert "look_left" not in docs_text
    assert "look_right" not in docs_text
