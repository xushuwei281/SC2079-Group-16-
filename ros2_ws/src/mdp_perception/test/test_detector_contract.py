"""Check model-loader failure behavior and PT marker filtering with small fakes."""

import importlib.util
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from mdp_perception.class_contract import _CANONICAL_MODEL_NAMES


def load_detector_type():
    """Import an isolated detector module without native inference packages."""
    source = Path(__file__).parents[1] / "mdp_perception" / "detector.py"
    spec = importlib.util.spec_from_file_location("isolated_contract_detector", source)
    module = importlib.util.module_from_spec(spec)
    with patch.dict("sys.modules", {"cv2": SimpleNamespace(), "numpy": SimpleNamespace()}):
        spec.loader.exec_module(module)
    return module.TargetDetector


class TestDetectorContract(unittest.TestCase):
    """Ensure invalid models fail and markers never become Android IDs."""

    @classmethod
    def setUpClass(cls):
        cls.detector_type = load_detector_type()

    def test_missing_requested_model_fails(self):
        with tempfile.TemporaryDirectory() as directory:
            requested = Path(directory) / "missing.onnx"
            with self.assertRaises(FileNotFoundError):
                self.detector_type(model_path=str(requested))

    def test_missing_onnx_does_not_choose_neighbor_pt(self):
        with tempfile.TemporaryDirectory() as directory:
            (Path(directory) / "best.pt").write_bytes(b"placeholder")
            with self.assertRaises(FileNotFoundError):
                self.detector_type(model_path=str(Path(directory) / "best.onnx"))

    def test_invalid_onnx_does_not_fall_back(self):
        def fail_session(*args, **kwargs):
            raise RuntimeError("bad ONNX artifact")

        ort = SimpleNamespace(
            SessionOptions=SimpleNamespace,
            GraphOptimizationLevel=SimpleNamespace(ORT_ENABLE_ALL=1),
            InferenceSession=fail_session,
        )
        with tempfile.TemporaryDirectory() as directory:
            model_path = Path(directory) / "best.onnx"
            model_path.write_bytes(b"placeholder")
            with patch.dict("sys.modules", {"onnxruntime": ort}):
                with self.assertRaisesRegex(RuntimeError, "bad ONNX artifact"):
                    self.detector_type(model_path=str(model_path))

    def test_pt_marker_filtered_but_filled_circle_retained(self):
        marker = SimpleNamespace(cls=[SimpleNamespace(item=lambda: 30)],
                                 conf=[SimpleNamespace(item=lambda: 0.99)])
        coordinates = SimpleNamespace(cpu=lambda: SimpleNamespace(
            numpy=lambda: SimpleNamespace(astype=lambda dtype: [1, 2, 30, 40])))
        circle = SimpleNamespace(cls=[SimpleNamespace(item=lambda: 29)],
                                 conf=[SimpleNamespace(item=lambda: 0.90)],
                                 xyxy=[coordinates])
        detector = object.__new__(self.detector_type)
        detector._onnx_session = None
        detector._pt_model = SimpleNamespace(predict=lambda **kwargs: [
            SimpleNamespace(boxes=[marker, circle])])
        detector._class_names = list(_CANONICAL_MODEL_NAMES)
        detector.conf_threshold = 0.5
        detector.iou_threshold = 0.45
        self.assertEqual(detector.predict(None), [("40", 40, 0.90, (1, 2, 30, 40))])

    def test_absent_backend_is_not_mock_mode(self):
        detector = object.__new__(self.detector_type)
        detector._onnx_session = None
        detector._pt_model = None
        with self.assertRaisesRegex(RuntimeError, "No validated inference backend"):
            detector.predict(None)


if __name__ == "__main__":
    unittest.main()
