import numpy as np
import pytest

cv2 = pytest.importorskip("cv2")

from project006.config import DEFAULTS  # noqa: E402
from project006.vision import detect_target  # noqa: E402


def _frame():
    return np.full((360, 640, 3), (90, 140, 60), np.uint8)   # grass-ish BGR


def test_detects_red_target_center():
    img = _frame()
    cv2.circle(img, (450, 120), 20, (0, 0, 230), -1)          # red disc (BGR)
    det = detect_target(img, DEFAULTS["tracking"]["hsv_ranges"], 150)
    assert det is not None
    assert det["u"] == pytest.approx(450, abs=1.5) and det["v"] == pytest.approx(120, abs=1.5)


def test_detects_red_on_the_wraparound_hue():
    img = _frame()
    cv2.circle(img, (100, 300), 20, (60, 0, 230), -1)          # magenta-red, hue ~173
    assert detect_target(img, DEFAULTS["tracking"]["hsv_ranges"], 150) is not None


def test_ignores_small_specks_and_empty_frames():
    img = _frame()
    assert detect_target(img, DEFAULTS["tracking"]["hsv_ranges"], 150) is None
    cv2.circle(img, (300, 200), 3, (0, 0, 230), -1)
    assert detect_target(img, DEFAULTS["tracking"]["hsv_ranges"], 150) is None
