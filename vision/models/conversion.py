'''
# the simple ultralytics way to convert to rpk(literally doesn't work)
import torch
from ultralytics import YOLO
model = YOLO('2-16-2026/v11s_2-16.pt')
model.export(format='imx')'''

'''
# the simple ultralytics way to convert to onnx(literally doesn't work)
import torch
from ultralytics import YOLO
model = YOLO('2-16-2026/v11s_2-16.pt')
model.export(format='onnx')'''

# the other way to convert to .rpk
import model_compression_toolkit as mct
import imx500_converter


