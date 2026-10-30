# Copyright 2026 EBVA (Escuderia Brasileira de Veículos Autônomos)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Device contracts without downloading weights or requiring CUDA hardware."""
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import MagicMock, patch

import numpy as np
from ebva_sign_detection.backend import YoloBackend


class TestBackend(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.path = Path(self.temp.name) / 'fixture.pt'
        self.path.write_bytes(b'fake checkpoint')
        self.torch = MagicMock(__version__='test-torch')
        self.torch.version.cuda = None
        self.model = MagicMock()
        self.model.names = {0: 'sign'}
        self.model.model.parameters.side_effect = lambda: iter(
            [SimpleNamespace(device='cpu')])
        self.ultralytics = SimpleNamespace(
            __version__='test-yolo',
            YOLO=MagicMock(
                return_value=self.model))
        patcher = patch.dict(
            'sys.modules',
            torch=self.torch,
            ultralytics=self.ultralytics)
        patcher.start()
        self.addCleanup(patcher.stop)

    def test_cpu_never_queries_or_synchronizes_cuda(self):
        self.torch.cuda.is_available.side_effect = AssertionError(
            'CPU queried CUDA')
        self.torch.cuda.synchronize.side_effect = AssertionError(
            'CPU synchronized CUDA')
        backend = YoloBackend(self.path, 'cpu', 32, .25)
        backend.predict(np.zeros((32, 32, 3), dtype=np.uint8))
        self.assertEqual(backend.metadata['device'], 'cpu')
        self.assertIsNone(backend.metadata['gpu'])
        self.model.to.assert_called_once_with('cpu')
        self.assertEqual(self.model.predict.call_args.kwargs['device'], 'cpu')
        self.torch.cuda.get_device_name.assert_not_called()

    def test_cuda_unavailable_fails_before_loading_model(self):
        self.torch.cuda.is_available.return_value = False
        with self.assertRaisesRegex(
                RuntimeError, 'CUDA selected but unavailable'):
            YoloBackend(self.path, 'cuda:0', 32, .25)
        self.ultralytics.YOLO.assert_not_called()

    def test_silent_device_fallback_is_rejected(self):
        self.torch.cuda.is_available.return_value = True
        with self.assertRaisesRegex(RuntimeError, 'device mismatch'):
            YoloBackend(self.path, 'cuda:0', 32, .25)

    def test_cuda_synchronizes_and_reports_effective_device(self):
        self.torch.cuda.is_available.return_value = True
        self.torch.cuda.get_device_name.return_value = 'test GPU'
        self.model.model.parameters.side_effect = lambda: iter(
            [SimpleNamespace(device='cuda:0')])
        backend = YoloBackend(self.path, 'cuda:0', 32, .25)
        self.assertEqual(backend.metadata['device'], 'cuda:0')
        self.assertEqual(backend.metadata['gpu'], 'test GPU')
        self.assertEqual(self.torch.cuda.synchronize.call_count, 2)
