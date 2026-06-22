from pathlib import Path


def test_keyboard_declares_explicit_circle_exit():
    source = Path(__file__).parents[1] / 'scripts' / 'teleop_keyboard.py'
    text = source.read_text(encoding='utf-8')
    assert "'/robot1/circle_exit'" in text
    assert "elif key == 'e':" in text
