import torch
from ultralytics import YOLO

# Încarcă modelul Ultralytics
model = YOLO("yolo11n-pose.pt")

# Creează un dummy input
dummy_input = torch.randn(1, 3, 640, 640)

# Exportă ca TorchScript
traced_model = torch.jit.trace(model.model, dummy_input)
traced_model.save("yolo11n-pose-ts.pt")
