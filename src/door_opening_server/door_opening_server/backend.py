"""FP32 ONNX inference with the exact RGB letterbox used for training."""
import hashlib
import json
from pathlib import Path
import numpy as np
from PIL import Image


def image_to_rgb(message):
    """Read sensor_msgs/Image without cv_bridge; honor encoding and row padding."""
    channels = {'rgb8': 3, 'bgr8': 3, 'rgba8': 4, 'bgra8': 4}.get(message.encoding)
    if channels is None:
        raise ValueError('Unsupported RGB image encoding: ' + message.encoding)
    height, width, step = int(message.height), int(message.width), int(message.step)
    if height <= 0 or width <= 0 or step < width * channels:
        raise ValueError('Invalid RGB image dimensions or row stride')
    buffer = memoryview(message.data)
    if buffer.nbytes != step * height:
        raise ValueError('RGB image buffer does not match height * step')
    array = np.ndarray((height, width, channels), dtype=np.uint8,
                       buffer=buffer, strides=(step, channels, 1))[:, :, :3]
    if message.encoding in ('bgr8', 'bgra8'):
        array = array[:, :, ::-1]
    return np.ascontiguousarray(array)


def tensor_rgb(rgb, size=640):
    if not isinstance(rgb, np.ndarray) or rgb.dtype != np.uint8 or rgb.ndim != 3 or rgb.shape[2] != 3:
        raise ValueError('Expected HxWx3 uint8 RGB')
    image = Image.fromarray(rgb)
    width, height = image.size
    if min(width, height) <= 0:
        raise ValueError('Empty RGB image')
    scale = min(size / width, size / height)
    resized = image.resize((max(1, round(width * scale)), max(1, round(height * scale))), Image.Resampling.BILINEAR)
    boxed = Image.new('RGB', (size, size), (114, 114, 114))
    boxed.paste(resized, ((size - resized.width) // 2, (size - resized.height) // 2))
    array = np.asarray(boxed, dtype=np.float32) / 255.0
    return np.ascontiguousarray(array.transpose(2, 0, 1)[None])


class OnnxDoorClassifier:
    def __init__(self, model_path, provider='auto', threads=4):
        import onnxruntime as ort
        path = Path(model_path)
        self.metadata = json.loads(path.with_suffix('.json').read_text())
        if self.metadata['epoch'] != 11 or self.metadata['threshold'] != .47:
            raise ValueError('Expected the user-selected epoch 11 model with threshold 0.47')
        digest = hashlib.sha256(path.read_bytes()).hexdigest()
        if digest != self.metadata['onnx_sha256']:
            raise ValueError('ONNX SHA256 does not match its deployment manifest')
        self.threshold = self.metadata['threshold']
        self.image_size = self.metadata['image_size']
        available = ort.get_available_providers()
        if provider == 'auto':
            requested = 'CUDAExecutionProvider' if 'CUDAExecutionProvider' in available else 'CPUExecutionProvider'
        else:
            requested = {'cpu':'CPUExecutionProvider', 'cuda':'CUDAExecutionProvider'}[provider]
        if requested not in available:
            raise RuntimeError(f'{requested} is unavailable; installed providers: {available}')
        options = ort.SessionOptions()
        options.intra_op_num_threads = threads
        options.inter_op_num_threads = 1
        providers = [requested] if requested == 'CPUExecutionProvider' else [requested, 'CPUExecutionProvider']
        self.session = ort.InferenceSession(str(path), sess_options=options, providers=providers)
        self.provider = self.session.get_providers()[0]
        if self.provider != requested:
            raise RuntimeError(f'ONNX Runtime fell back from {requested} to {self.provider}')
        inputs, outputs = self.session.get_inputs(), self.session.get_outputs()
        if len(inputs) != 1 or inputs[0].name != 'rgb' or inputs[0].shape != [1,3,640,640]:
            raise ValueError('Unexpected ONNX RGB input schema')
        if len(outputs) != 1 or outputs[0].name != 'probabilities':
            raise ValueError('Unexpected ONNX output schema')
        # Initialize kernels before advertising the service. Warmup decisions are discarded.
        self.score(np.zeros((360,640,3), dtype=np.uint8))

    def score(self, rgb) -> float:
        output = self.session.run(['probabilities'], {'rgb':tensor_rgb(rgb, self.image_size)})[0]
        if output.shape != (1,2) or not np.isfinite(output).all():
            raise ValueError('Invalid/nonfinite model output')
        if not ((output >= 0).all() and (output <= 1).all()) or abs(float(output.sum()) - 1) > 1e-4:
            raise ValueError('Model did not return two class probabilities')
        return float(output[0,1])
