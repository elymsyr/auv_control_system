import torch
import sys

# --- Configuration ---
# Path to your saved PyTorch scripted model
pytorch_model_path = 'models/fossen_net_1/fossen_net_scripted.pt'
# Desired path for the output ONNX model
onnx_model_path = 'models/fossen_net_1/fossen_net.onnx'

# From your C++ code (rawInputData.size() != 501), we know the input feature size is 501.
# The shape for a single prediction is (batch_size, num_features).
input_shape = (1, 501)
# --- End Configuration ---

print(f"Loading PyTorch model from: {pytorch_model_path}")
try:
    model = torch.jit.load(pytorch_model_path)
    
    # --- THIS IS THE FIX ---
    # Explicitly move the model and all its parameters to the CPU.
    model.to('cpu')
    # --- END OF FIX ---

    model.eval() # Set the model to evaluation mode (important!)
except Exception as e:
    print(f"Error loading model: {e}")
    sys.exit(1)

# Create a dummy input tensor. By default, this is created on the CPU,
# which now matches the model's location.
dummy_input = torch.randn(input_shape, requires_grad=False)

print(f"Exporting model to ONNX at: {onnx_model_path}")

# Export the model
torch.onnx.export(model,               # The model to export
                  dummy_input,         # A dummy input for tracing the model's graph
                  onnx_model_path,     # Where to save the model
                  export_params=True,  # Store the trained weights in the model file
                  opset_version=11,    # A good default ONNX version
                  do_constant_folding=True, # Execute constant folding for optimization
                  input_names = ['input'],   # The model's input names
                  output_names = ['output']) # The model's output names

print("\nExport successful!")
print(f"You can now use '{onnx_model_path}' in your C++ code.")