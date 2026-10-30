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

"""YOLO backend shared by CPU and CUDA ROS execution."""

import hashlib
import time

import numpy as np


class YoloBackend:
    """Load and warm the requested checkpoint before accepting ROS images."""

    def __init__(self, path, device, imgsz, confidence):
        import torch
        import ultralytics

        if device not in ('cpu', 'cuda:0'):
            raise ValueError('device must be cpu or cuda:0')
        if device == 'cuda:0' and not torch.cuda.is_available():
            raise RuntimeError(
                'CUDA selected but unavailable; check host driver '
                'and container GPU access')
        self.device = device
        self.torch = torch
        self.model = ultralytics.YOLO(str(path), task='detect')
        self.model.to(device)
        self.options = dict(
            device=device, imgsz=imgsz, conf=confidence,
            save=False, verbose=False)
        self.predict(np.zeros((imgsz, imgsz, 3), dtype=np.uint8))
        effective = str(next(self.model.model.parameters()).device)
        if effective != device:
            raise RuntimeError(
                f'YOLO device mismatch: requested {device}, '
                f'effective {effective}')
        self.metadata = dict(
            requested_device=device, device=effective,
            gpu=torch.cuda.get_device_name(0) if device == 'cuda:0' else None,
            torch=str(torch.__version__),
            ultralytics=str(ultralytics.__version__),
            cuda_runtime=torch.version.cuda,
            model_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
            names=self.model.names)

    def predict(self, frame):
        """Measure a synchronized model call, excluding ROS and encoding."""
        if self.device == 'cuda:0':
            self.torch.cuda.synchronize()
        started = time.perf_counter()
        result = self.model.predict(source=frame, **self.options)[0]
        if self.device == 'cuda:0':
            self.torch.cuda.synchronize()
        return result, (time.perf_counter() - started) * 1000
