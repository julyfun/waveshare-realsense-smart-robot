from typing import List, Optional

def unwrap(value, msg="Unwrap failed - value is None"):
    if value is None:
        raise ValueError(msg)
    return value
