"""Validation helpers for the scan dashboard."""

import json
from typing import Dict, Optional, Tuple

from .nav_protocol import (
    SCAN_CONTEXT_BED1,
    SCAN_CONTEXT_BED3,
    SCAN_CONTEXT_ORDER,
)


VALID_CONTEXTS = frozenset(
    (SCAN_CONTEXT_ORDER, SCAN_CONTEXT_BED1, SCAN_CONTEXT_BED3)
)


def parse_scan_result(payload: str) -> Optional[Tuple[int, str]]:
    try:
        data = json.loads(payload)
        context = int(data["context"])
        value = str(data["value"]).strip()
    except (KeyError, TypeError, ValueError, json.JSONDecodeError):
        return None
    if context not in VALID_CONTEXTS or not value:
        return None
    return context, value


def parse_scan_status(payload: str) -> Optional[Dict[int, str]]:
    try:
        values = json.loads(payload)["scan_values"]
        parsed = {
            context: str(values.get(str(context), values.get(context, ""))).strip()
            for context in VALID_CONTEXTS
        }
    except (AttributeError, KeyError, TypeError, ValueError, json.JSONDecodeError):
        return None
    return parsed
