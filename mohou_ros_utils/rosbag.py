from typing import Dict, List 

import rospy
from rosbag import Bag

from mohou_ros_utils.synclonizer import TimeStampedSequence
from mohou_ros_utils.synclonizer import synclonize


def bag_to_seqs(bag: Bag) -> List[TimeStampedSequence]:

    table: Dict[str, TimeStampedSequence] = {}
    for topic_name, msg, time in bag.read_messages():  # type: ignore
        if not topic_name in table:
            table[topic_name] = TimeStampedSequence.create_empty(type(msg), topic_name=topic_name)
        table[topic_name].append(msg, time.to_sec())

    return list(table.values())


def bag_to_synced_seqs(bag: Bag, freq: float) -> List[TimeStampedSequence]:
    seqs = bag_to_seqs(bag)
    seqs_sync = synclonize(seqs, freq)
    return seqs_sync
