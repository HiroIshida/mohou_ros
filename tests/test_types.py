from mohou_ros_utils.types import TimeStampedSequence, TimeStampedSequenceChunk


def test_chunk():
    seq1 = TimeStampedSequence.create_empty(int)
    seq2 = TimeStampedSequence.create_empty(str)
    chunk = TimeStampedSequenceChunk([seq1, seq2])
    assert chunk.is_type_to_list_injective()

    seq_int = chunk.filter_by_type(int)
    assert seq_int.object_type == int

    seq1 = TimeStampedSequence.create_empty(int)
    seq2 = TimeStampedSequence.create_empty(int)
    chunk = TimeStampedSequenceChunk([seq1, seq2])
    assert not chunk.is_type_to_list_injective()
