from types import SimpleNamespace
import numpy as np
import pytest
from door_opening_server.backend import image_to_rgb
from door_opening_server.policy import SamplingWindow, opening_percent


def test_mean_probability_decision_accepts_one_to_five_frames():
    assert opening_percent([.99]*5,.47)==100.0
    assert opening_percent([.47]*5,.47)==0.0
    for count in range(1,6):
        assert opening_percent([.48]*count,.47)==100.0
        assert opening_percent([.46]*count,.47)==0.0
    for red in range(5):
        scores=[.99]*5;scores[red]=.469
        assert opening_percent(scores,.47)==100.0
    assert opening_percent([.8,.1],.47)==0.0
    assert opening_percent([.9,.1],.47)==100.0
    for invalid in ([], [.99]*6, [float('nan')]*5, [float('inf')]*5, [-1.]*5, [1.01]):
        assert opening_percent(invalid,.47)==0.0


@pytest.mark.parametrize('count',[1,2,3,4])
def test_sparse_window_selects_all_available_valid_frames(count):
    window=SamplingWindow(1.,1_000_000_000,250_000_000)
    assert not window.complete()
    for index in range(count):
        stamp=1_100_000_000+index*10_000_000
        assert window.offer(index,stamp,1.1+index*.01,stamp)
    assert window.complete()
    assert len(window.selected())==count


def test_latest_frame_from_each_bin_and_no_stale_reuse():
    start=10_000_000_000
    window=SamplingWindow(100.,start,250_000_000)
    for index in range(30):
        elapsed_ns=10_000_000+index*33_000_000
        window.offer(index,start+elapsed_ns,100.+elapsed_ns/1e9,start+elapsed_ns+1_000_000)
    assert window.complete()
    assert [x.image for x in window.selected()]==[0,7,14,22,29]
    new=SamplingWindow(102.,start+2_000_000_000,250_000_000)
    assert not new.offer('old',start+999_000_000,102.01,start+2_010_000_000)
    assert not new.complete()


def test_five_distinct_frames_do_not_require_artificial_time_bins():
    window=SamplingWindow(1.,1_000_000_000,250_000_000)
    for index in range(5):
        stamp=1_010_000_000+index*10_000_000
        assert window.offer(index,stamp,1.01+index*.01,stamp)
    assert window.complete() and len(window.selected())==5


def test_stale_repeated_out_of_order_zero_and_late_stamps_rejected():
    window=SamplingWindow(1.,1_000_000_000,250_000_000)
    assert not window.offer('zero',0,1.1,1_100_000_000)
    assert not window.offer('stale',1_010_000_000,1.5,1_500_000_000)
    assert not window.offer('before',999_000_000,1.1,1_100_000_000)
    assert not window.offer('future',1_110_000_000,1.1,1_100_000_000)
    assert window.offer('first',1_100_000_000,1.11,1_110_000_000)
    assert not window.offer('repeat',1_100_000_000,1.2,1_200_000_000)
    assert not window.offer('out-of-order',1_090_000_000,1.2,1_200_000_000)
    assert not window.offer('late',1_999_000_000,2.0,2_000_000_000)


@pytest.mark.parametrize('encoding,channels',[('rgb8',3),('bgr8',3),('rgba8',4),('bgra8',4)])
def test_rgb_channel_order_and_row_padding(encoding,channels):
    rgb=np.array([[[255,3,9],[12,77,4]],[[1,2,3],[4,5,6]]],dtype=np.uint8)
    pixels=rgb[:,:,::-1] if encoding.startswith('bgr') else rgb
    if channels==4:pixels=np.concatenate([pixels,np.full((2,2,1),255,dtype=np.uint8)],axis=2)
    padded=np.full((2,2*channels+7),198,dtype=np.uint8)
    padded[:,:2*channels]=pixels.reshape(2,-1)
    message=SimpleNamespace(width=2,height=2,step=padded.shape[1],encoding=encoding,data=padded.tobytes())
    np.testing.assert_array_equal(image_to_rgb(message),rgb)
    message.data=message.data[:-1]
    with pytest.raises(ValueError):image_to_rgb(message)


def test_depth_and_unknown_encodings_are_rejected():
    for encoding in ('16UC1','32FC1','mono8','yuv422'):
        with pytest.raises(ValueError):image_to_rgb(SimpleNamespace(encoding=encoding))
