from dataclasses import dataclass
from typing import Dict, List, Any, Optional, Type, TypeVar, Generic


ObjectT = TypeVar('ObjectT')


@dataclass(frozen=True)
class TimeStampedSequence(Generic[ObjectT]):
    object_type: Type[ObjectT]
    object_list: List[Optional[ObjectT]]
    time_list: List[float]
    topic_name: Optional[str] = None

    @classmethod
    def create_empty(cls, object_type: Type[ObjectT], topic_name = None):
        return cls(object_type, [], [], topic_name)

    def append(self, obj: Optional[ObjectT], time: float):
        self.object_list.append(obj)
        self.time_list.append(time)

    def __len__(self):
        return len(self.object_list)

    def __post_init__(self):
        assert len(self.object_list) == len(self.time_list)
