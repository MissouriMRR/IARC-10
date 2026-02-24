import torch
from ultralytics import YOLO

# The simple Ultralytics way - just do this:
model = YOLO('2-16-2026/v11s_2-16.pt')
model.export(format='imx')