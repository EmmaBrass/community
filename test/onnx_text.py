import onnx

# Path to your ONNX model
model_path = '/home/emma/piper-voices/en/en_GB/alan/medium/en_GB-alan-medium.onnx'

# Load and check the model
model = onnx.load(model_path)
onnx.checker.check_model(model)

print("The model is valid!")