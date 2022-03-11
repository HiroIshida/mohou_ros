import re
from collections.abc import Sequence

from dataclasses import dataclass
from typing import List, Optional, Type, TypeVar, Generic


ObjectT = TypeVar('ObjectT')


@dataclass
class TimeStampedSequence(Generic[ObjectT]):
    object_type: Type[ObjectT]
    object_list: List[Optional[ObjectT]]
    time_list: List[float]
    topic_name: Optional[str] = None

    @classmethod
    def create_empty(cls, object_type: Type[ObjectT], topic_name=None):
        return cls(object_type, [], [], topic_name)

    def append(self, obj: Optional[ObjectT], time: float):
        self.object_list.append(obj)
        self.time_list.append(time)

    def __len__(self):
        return len(self.object_list)

    def __post_init__(self):
        assert len(self.object_list) == len(self.time_list)

    def is_valid(self):
        return not (None in self.object_list)


@dataclass
class TimeStampedSequenceChunk(Sequence):
    tss_list: List[TimeStampedSequence]

    def is_type_to_list_injective(self) -> bool:
        type_set = set([tss.object_type for tss in self.tss_list])
        return len(type_set) == len(self.tss_list)

    def filter_by_type(self, type_query: ObjectT) -> List[TimeStampedSequence[ObjectT]]:

        tss_list_out: List[TimeStampedSequence[ObjectT]] = []

        for tss in self.tss_list:
            if tss.object_type == type_query:
                tss_list_out.append(tss)
        return tss_list_out

    def filter_by_topic_name(self, topic_name_query: str) -> List[TimeStampedSequence]:
        tss_list_out = []
        for tss in self.tss_list:
            if tss.topic_name is None:
                continue
            m = re.match(r".*{}.*".format(topic_name_query), tss.topic_name)
            if m is not None:
                tss_list_out.append(tss)
        return tss_list_out

    def __getitem__(self, index):
        return self.tss_list[index]

    def __len__(self) -> int:
        return len(self.tss_list)
