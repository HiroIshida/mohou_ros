import pytest
import numpy as np

from mohou_ros_utils.synclonizer import TimeStampedSequnce
from mohou_ros_utils.synclonizer import synclonize


def test_synclonize():
    N = 6
    times = np.array([float(i) for i in range(N)])
    times2 = times + 0.2
    seq1 = TimeStampedSequnce([None for _ in range(N)], times)
    seq2 = TimeStampedSequnce([None for _ in range(N)], times2)

    with pytest.raises(AssertionError):
        synclonize([seq1, seq2], freq=0.6)

    seqs_new = synclonize([seq1, seq2], freq=1.0 + 1e-12)
    seq1_new, seq2_new = seqs_new
    assert abs(seq1_new.times[0] - 1.0) < 1e-4
    assert abs(seq1_new.times[-1] - (N-1.0)) < 1e-4

    seqs_new = synclonize([seq1, seq2], freq=2.0 + 1e-12)
    seq1_new, seq2_new = seqs_new
    assert abs(seq1_new.times[0] - 2.0) < 1e-4
    assert abs(seq1_new.times[-1] - N) < 1e-4
