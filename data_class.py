from pydantic import BaseModel
from typing import List

class Flts(BaseModel):
    flts: List[float]
