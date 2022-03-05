import pytest
import numpy as np

from mohou_ros_utils.synclonizer import TimeStampedSequence
from mohou_ros_utils.synclonizer import synclonize


def assert_float_almost_equal(a, b):
    assert abs(a - b) < 1e-4


def test_synclonize_case1():
    N = 6
    times = np.array([float(i) for i in range(N)])
    times2 = times + 0.2
    seq1 = TimeStampedSequence([None for _ in range(N)], times)
    seq2 = TimeStampedSequence([None for _ in range(N)], times2)

    with pytest.raises(AssertionError):
        synclonize([seq1, seq2], freq=0.6)

    seqs_new = synclonize([seq1, seq2], freq=1.0 + 1e-12)
    seq1_new, seq2_new = seqs_new
    assert_float_almost_equal(seq1_new.time_list[0], 0.5)
    assert_float_almost_equal(seq1_new.time_list[-1], (N-1.5))

    seqs_new = synclonize([seq1, seq2], freq=2.0 + 1e-12)
    seq1_new, seq2_new = seqs_new
    assert_float_almost_equal(seq1_new.time_list[0], 1.0)
    assert_float_almost_equal(seq1_new.time_list[-1], N - 1.0)


def test_synclonize_case2():
    N = 12
    times = np.array([float(i) * 1.0 for i in range(N)])
    times2 = np.array([float(i) * 1.2 for i in range(N)])
    seq1 = TimeStampedSequence([None for _ in range(N)], times)
    seq2 = TimeStampedSequence([None for _ in range(N)], times2)

    freq = 2.0
    seqs_new = synclonize([seq1, seq2], freq=freq)
    assert len(seqs_new[0].time_list) == 6
