import re
from typing import Dict, List, Tuple

from rosbag import Bag

from mohou_ros_utils.synclonizer import TimeStampedSequence
from mohou_ros_utils.synclonizer import synclonize
from mohou_ros_utils.interpolator import AbstractInterpolationRule, NullInterpolationRule


def resolve_topic_type(tmp_msg_type_name: str) -> Tuple[str, str]:
    """convert tmpu7d7puwo._std_msgs__String -> std_msgs.msg, String"""
    m = re.match(r"(\w+)._(\w+)__(\w+)", tmp_msg_type_name)
    assert m is not None
    module_name = m.group(2) + '.msg'
    message_name = m.group(3)
    return module_name, message_name


def bag_to_seqs(bag: Bag) -> List[TimeStampedSequence]:

    table: Dict[str, TimeStampedSequence] = {}
    for topic_name, msg, time in bag.read_messages():  # type: ignore
        if topic_name not in table:
            table[topic_name] = TimeStampedSequence.create_empty(type(msg), topic_name=topic_name)
        table[topic_name].append(msg, time.to_sec())

    return list(table.values())


def bag_to_synced_seqs(
        bag: Bag,
        freq: float,
        rule: AbstractInterpolationRule = NullInterpolationRule()) -> List[TimeStampedSequence]:
    seqs = bag_to_seqs(bag)
    seqs_sync = synclonize(seqs, freq, rule)
    return seqs_sync
