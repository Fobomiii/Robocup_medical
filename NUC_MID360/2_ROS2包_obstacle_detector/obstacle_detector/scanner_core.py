"""Pure validation and temporal confirmation for competition code scanning."""

from collections import defaultdict
from typing import DefaultDict, List, Optional, Tuple

from .nav_protocol import (
    SCAN_CONTEXT_BED1,
    SCAN_CONTEXT_BED3,
    SCAN_CONTEXT_ORDER,
    SCAN_FORMAT_CODE128,
    SCAN_FORMAT_QR,
)


TASK_SCAN_ORDER = 2
TASK_SCAN_BED1 = 4
TASK_SCAN_BED3 = 7

QR_VALUES = frozenset(("11", "13", "31", "33"))
CODE128_VALUES = frozenset(
    (
        "6946522463487",
        "6921361255288",
        "6911345321863",
        "6944060407291",
        "6906841121017",
        "6938237700261",
    )
)


def expected_scan(task_state: int) -> Optional[Tuple[int, int]]:
    if task_state == TASK_SCAN_ORDER:
        return SCAN_CONTEXT_ORDER, SCAN_FORMAT_QR
    if task_state == TASK_SCAN_BED1:
        return SCAN_CONTEXT_BED1, SCAN_FORMAT_CODE128
    if task_state == TASK_SCAN_BED3:
        return SCAN_CONTEXT_BED3, SCAN_FORMAT_CODE128
    return None


def value_is_allowed(format: int, value: str) -> bool:
    if format == SCAN_FORMAT_QR:
        return value in QR_VALUES
    if format == SCAN_FORMAT_CODE128:
        return value in CODE128_VALUES
    return False


def scan_matches_task(task_state: int, context: int, format: int, value: str) -> bool:
    expected = expected_scan(task_state)
    return expected == (context, format) and value_is_allowed(format, value)


class ScanConsensus:
    def __init__(self, required_hits: int = 2, window_s: float = 0.8) -> None:
        if required_hits < 1 or window_s <= 0.0:
            raise ValueError("invalid scan consensus configuration")
        self.required_hits = int(required_hits)
        self.window_s = float(window_s)
        self._hits: DefaultDict[Tuple[int, str], List[float]] = defaultdict(list)
        self._latched: Optional[Tuple[int, str]] = None

    def reset(self) -> None:
        self._hits.clear()
        self._latched = None

    def observe(self, format: int, value: str, now_s: float) -> bool:
        key = (format, value)
        if self._latched == key:
            return False
        cutoff = now_s - self.window_s
        recent = [stamp for stamp in self._hits[key] if stamp >= cutoff]
        recent.append(now_s)
        self._hits[key] = recent
        if len(recent) < self.required_hits:
            return False
        self._latched = key
        return True
