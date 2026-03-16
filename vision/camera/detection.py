from picamera2.devices import IMX500

class Detection:
	def __init__(self, coords, category, conf, metadata):
		self.category = category # detected object type
		self.confidence = conf
		self.box = imx500.convert_inference_coords(coords, metadata, picam2)
		if category is not None and PRINT_DETECTIONS:
			print(f"{LABELS[int(category)]}? - {conf * 100:.2f}%")