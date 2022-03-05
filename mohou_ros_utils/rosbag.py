import re
from typing import Dict, List, Tuple

from rosbag import Bag

from mohou_ros_utils.synclonizer import TimeStampedSequence
from mohou_ros_utils.synclonizer import synclonize
from mohou_ros_utils.interpolator import AbstractInterpolationRule, NullInterpolationRule


def resolve_topic_type(tmp_msg_type_name: str) -> Tuple[str, str]:
    """convert _std_msgs__String -> std_msgs.msg, String"""
    m = re.match(r"_(\w+)__(\w+)", tmp_msg_type_name)
    assert m is not None
    module_name = m.group(1) + '.msg'
    message_name = m.group(2)
    return module_name, message_name


def bag_to_seqs(bag: Bag) -> List[TimeStampedSequence]:

    table: Dict[str, TimeStampedSequence] = {}
    for topic_name, msg, time in bag.read_messages():  # type: ignore
        if topic_name not in table:
            module_name, type_name = resolve_topic_type(msg.__class__.__name__)
            ldict = {'message_type': None}
            try:
                exec('message_type = {}'.format(type_name), globals(), ldict)
            except NameError:
                # https://stackoverflow.com/questions/1463306
                # These code seems to dangerous. Is there better way to do this?
                exec('from {} import {}'.format(module_name, type_name), globals(), ldict)
                exec('message_type = {}'.format(type_name), globals(), ldict)
            message_type = ldict['message_type']
            assert message_type is not None
            table[topic_name] = TimeStampedSequence.create_empty(message_type, topic_name=topic_name)  # type: ignore
        table[topic_name].append(msg, time.to_sec())

    return list(table.values())


def bag_to_synced_seqs(
        bag: Bag,
        freq: float,
        rule: AbstractInterpolationRule = NullInterpolationRule()) -> List[TimeStampedSequence]:
    seqs = bag_to_seqs(bag)
    seqs_sync = synclonize(seqs, freq, rule)
    return seqs_sync
