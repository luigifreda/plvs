"""Exercise the existing alignment math and NumPy scalar/array conversions."""
import importlib.util
from pathlib import Path
import sys
import subprocess
import tempfile
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "Scripts"))


def load_alignment(relative_path):
    spec = importlib.util.spec_from_file_location("alignment_under_test", ROOT / relative_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.align


class AlignmentTests(unittest.TestCase):
    def setUp(self):
        self.model = np.matrix([[0, 1, 0, 0, 2], [0, 0, 1, 0, -1], [0, 0, 0, 1, 3]], dtype=float)
        self.rotation = np.matrix([[0, -1, 0], [1, 0, 0], [0, 0, 1]], dtype=float)
        self.translation = np.matrix([[1], [2], [3]], dtype=float)

    def test_rigid_alignment(self):
        align = load_alignment("Scripts/evaluate_ate.py")
        rotation, translation, errors = align(self.model, self.rotation * self.model + self.translation)
        np.testing.assert_allclose(rotation, self.rotation, atol=1e-12)
        np.testing.assert_allclose(translation, self.translation, atol=1e-12)
        np.testing.assert_allclose(errors, 0, atol=1e-12)
        self.assertEqual(errors.shape, (self.model.shape[1],))

    def test_scaled_alignment(self):
        for path in ("Scripts/evaluate_ate_scale.py", "evaluation/evaluate_ate_scale.py"):
            for expected_scale in (1.0, 2.5):
                with self.subTest(script=path, scale=expected_scale):
                    align = load_alignment(path)
                    data = expected_scale * self.rotation * self.model + self.translation
                    rotation, translation, errors, _, rigid_errors, scale = align(self.model, data)
                    np.testing.assert_allclose(rotation, self.rotation, atol=1e-12)
                    np.testing.assert_allclose(translation, self.translation, atol=1e-12)
                    np.testing.assert_allclose(errors, 0, atol=1e-12)
                    self.assertAlmostEqual(scale, expected_scale)
                    self.assertEqual(errors.shape, (self.model.shape[1],))
                    self.assertEqual(rigid_errors.shape, errors.shape)
                    if expected_scale == 1:
                        np.testing.assert_allclose(rigid_errors, 0, atol=1e-12)
                    else:
                        self.assertGreater(np.linalg.norm(rigid_errors), 0)

    def test_command_line_exports(self):
        expected = np.asarray((self.rotation * self.model + self.translation).T)
        timestamps = np.arange(self.model.shape[1], dtype=float)
        quaternions = np.tile([0, 0, 0, 1], (len(timestamps), 1))
        with tempfile.TemporaryDirectory() as directory:
            directory = Path(directory)
            groundtruth = directory / "groundtruth.txt"
            estimated = directory / "estimated.txt"
            np.savetxt(groundtruth, np.column_stack([timestamps, expected, quaternions]))
            np.savetxt(estimated, np.column_stack([timestamps, np.asarray(self.model.T), quaternions]))
            for script in ("Scripts/evaluate_ate.py", "Scripts/evaluate_ate_scale.py", "evaluation/evaluate_ate_scale.py"):
                with self.subTest(script=script):
                    saved = directory / "aligned.txt"
                    associations = directory / "associations.txt"
                    subprocess.run([sys.executable, str(ROOT / script), str(groundtruth), str(estimated),
                                    "--save", str(saved), "--save_associations", str(associations)],
                                   check=True, capture_output=True, text=True)
                    np.testing.assert_allclose(np.loadtxt(saved)[:, 1:4], expected, atol=1e-6)
                    np.testing.assert_allclose(np.loadtxt(associations)[:, 5:8], expected, atol=1e-6)


if __name__ == "__main__":
    unittest.main()
