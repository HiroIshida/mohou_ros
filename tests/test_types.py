from mohou_ros_utils.types import TimeStampedSequence, TimeStampedSequenceChunk


def test_chunk():
    seq1 = TimeStampedSequence.create_empty(int)
    seq2 = TimeStampedSequence.create_empty(str)
    chunk = TimeStampedSequenceChunk([seq1, seq2])
    assert chunk.is_type_to_list_injective()

    seq_int = chunk.filter_by_type(int)
    assert len(seq_int) == 1
    assert seq_int[0].object_type == int

    seq1 = TimeStampedSequence.create_empty(int, topic_name='hoge/depth_image')
    seq2 = TimeStampedSequence.create_empty(int, topic_name='hoge/rgb_image')
    seq3 = TimeStampedSequence.create_empty(int, topic_name='hoge/rgb_image_half')
    chunk = TimeStampedSequenceChunk([seq1, seq2, seq3])
    assert not chunk.is_type_to_list_injective()

    seqs_int = chunk.filter_by_type(int)
    assert len(seqs_int) == 3

    seqs = chunk.filter_by_topic_name('depth')
    assert len(seqs) == 1
    assert seqs[0].topic_name == seq1.topic_name

    seqs = chunk.filter_by_topic_name('rgb')
    assert len(seqs) == 2
    assert set([seq.topic_name for seq in seqs]) == set([seq2.topic_name, seq3.topic_name])
