"""Exercise the class contract without ROS, OpenCV, or model weights."""

import json
from types import SimpleNamespace
import unittest

from mdp_perception.class_contract import (
    _CANONICAL_MODEL_NAMES,
    _normalize_class_names,
    _parse_onnx_class_names,
    _resolve_symbol_id,
    _validate_model_names,
    _validate_onnx_contract,
)


def make_session(names=None, input_shape=None, output_shape=None, task="detect"):
    """Supply the metadata interface used by the contract validator."""
    metadata = {
        "names": repr(dict(enumerate(names or _CANONICAL_MODEL_NAMES))),
        "task": task,
    }
    return SimpleNamespace(
        get_modelmeta=lambda: SimpleNamespace(custom_metadata_map=metadata),
        get_inputs=lambda: [SimpleNamespace(
            name="images", type="tensor(float)",
            shape=input_shape or [1, 3, 640, 640])],
        get_outputs=lambda: [SimpleNamespace(
            name="output0", type="tensor(float)",
            shape=output_shape or [1, 35, 8400])],
    )


class TestClassContract(unittest.TestCase):
    """Pin target IDs, metadata interpretation, and runtime shape constraints."""

    def test_all_canonical_classes(self):
        self.assertEqual(
            [_resolve_symbol_id(name) for name in _CANONICAL_MODEL_NAMES],
            list(range(11, 41)) + [None],
        )

    def test_semantic_helper_compatibility(self):
        expected = {
            "1": 11, "9": 19, "a": 20, "d": 23, "s": 28,
            "z": 35, "up": 36, "down": 37, "right": 38,
            "left": 39, "circle": 40, "stop": 40, " A ": 20,
        }
        for name, symbol in expected.items():
            with self.subTest(name=name):
                self.assertEqual(_resolve_symbol_id(name), symbol)

    def test_markers_and_unknowns_are_excluded(self):
        for name in ("marker", "bullseye", "bulls_eye", "bulls-eye", "target",
                     "unknown", "41", "-1", "0", "10", ""):
            with self.subTest(name=name):
                self.assertIsNone(_resolve_symbol_id(name))

    def test_metadata_supported_encodings(self):
        names = list(_CANONICAL_MODEL_NAMES)
        encodings = [json.dumps(names), json.dumps(dict(enumerate(names))),
                     repr(dict(enumerate(names)))]
        for raw in encodings:
            self.assertEqual(_parse_onnx_class_names(raw), names)

    def test_dictionary_is_sorted_by_index(self):
        names = dict(reversed(list(enumerate(_CANONICAL_MODEL_NAMES))))
        self.assertEqual(_validate_model_names(names), list(_CANONICAL_MODEL_NAMES))

    def test_missing_malformed_or_wrong_metadata(self):
        for raw in (None, "", "no metadata", "{'broken':", "null", "42",
                    "__import__('os').system('echo unsafe')"):
            with self.subTest(raw=raw), self.assertRaises(ValueError):
                _parse_onnx_class_names(raw)

    def test_wrong_count_and_wrong_order(self):
        names = list(_CANONICAL_MODEL_NAMES)
        swapped = names.copy()
        swapped[0], swapped[1] = swapped[1], swapped[0]
        for wrong in (names[:-1], names + ["extra"], swapped,
                      ["1"] + names[1:], names[:-1] + ["target"]):
            with self.subTest(wrong=wrong), self.assertRaises(ValueError):
                _validate_model_names(wrong)

    def test_invalid_indices_and_value_types(self):
        for names in ({1: "11"}, {-1: "11"}, {0: "11", "0": "12"},
                      {True: "11"}, {0.0: "11"}, {"x": "11"}, [11], [""]):
            with self.subTest(names=names), self.assertRaises(ValueError):
                _normalize_class_names(names)

    def test_expected_static_detection_contract(self):
        self.assertEqual(_validate_onnx_contract(make_session()),
                         list(_CANONICAL_MODEL_NAMES))

    def test_wrong_input_contract(self):
        for shape in (["batch", 3, 640, 640], [1, 3, 320, 320], [2, 3, 640, 640]):
            with self.subTest(shape=shape), self.assertRaises(ValueError):
                _validate_onnx_contract(make_session(input_shape=shape))

    def test_wrong_output_contract(self):
        for shape in ([1, 300, 6], [1, 8400, 35], [1, 36, 8400], [1, 35, "anchors"]):
            with self.subTest(shape=shape), self.assertRaises(ValueError):
                _validate_onnx_contract(make_session(output_shape=shape))

    def test_wrong_task(self):
        with self.assertRaises(ValueError):
            _validate_onnx_contract(make_session(task="segment"))

    def test_wrong_input_dtype(self):
        session = make_session()
        session.get_inputs = lambda: [SimpleNamespace(
            name="images", type="tensor(float16)", shape=[1, 3, 640, 640])]
        with self.assertRaises(ValueError):
            _validate_onnx_contract(session)

    def test_missing_or_extra_tensors(self):
        for attribute in ("get_inputs", "get_outputs"):
            session = make_session()
            setattr(session, attribute, lambda: [])
            with self.assertRaises(ValueError):
                _validate_onnx_contract(session)


if __name__ == "__main__":
    unittest.main()
