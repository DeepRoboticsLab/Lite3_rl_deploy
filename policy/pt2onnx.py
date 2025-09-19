import torch
import numpy as np
import onnx
import onnxruntime as ort

# === Step 1: Load TorchScript model ===
# Example path to a pre-trained TorchScript model (for demonstration only)
pt_model_path = "model_528.pt"
print(f" Loading TorchScript model from {pt_model_path}")
ts_model = torch.jit.load(pt_model_path)
ts_model.eval()
                            
# === Step 2: Prepare dummy input ===
# Example observation dimension (for demonstration only)
obs_dim = 117
# Example number of historical observations (for demonstration only)
obs_his_num = 40
# Example total input dimension calculated from obs_dim and obs_his_num (for demonstration only)
obs_total_dim = obs_dim + obs_dim * obs_his_num  # 4817
print(f" Total ONNX input dimension: {obs_total_dim}")

# Example dummy input tensor with ones (for demonstration only)
dummy_input = torch.ones(1, obs_total_dim)

# === Step 3: Export to ONNX ===
# Example output path for ONNX model (for demonstration only)
onnx_model_path = "model_528.onnx"
torch.onnx.export(
    ts_model,
    dummy_input,
    onnx_model_path,
    input_names=["obs"],
    output_names=["action"],
    opset_version=11,
    dynamic_axes={
        "obs": {0: "batch_size"},
        "action": {0: "batch_size"},
    },
    verbose=False
)
print(f" Exported ONNX model to {onnx_model_path}")

# === Step 4: Verify ONNX model ===
# Example loading of the exported ONNX model (for demonstration only)
onnx_model = onnx.load(onnx_model_path)
onnx.checker.check_model(onnx_model)
print(" ONNX model structure is valid!")

# === Step 5: Run inference with both Torch and ONNX ===
print("\n Comparing outputs between TorchScript and ONNX...")

# TorchScript inference
with torch.no_grad():
    # Example TorchScript model output (for demonstration only)
    ts_output = ts_model(dummy_input).numpy()

# ONNXRuntime inference
# Example ONNX runtime session (for demonstration only)
ort_sess = ort.InferenceSession(onnx_model_path)
# Example input dictionary for ONNX model (for demonstration only)
ort_inputs = {"obs": dummy_input.numpy()}
# Example ONNX model output (for demonstration only)
ort_output = ort_sess.run(["action"], ort_inputs)[0]

# === Step 6: Compare outputs ===
# Example calculation of absolute difference between outputs (for demonstration only)
abs_diff = np.abs(ts_output - ort_output)
# Example maximum absolute difference (for demonstration only)
max_diff = np.max(abs_diff)
# Example mean absolute difference (for demonstration only)
mean_diff = np.mean(abs_diff)

print(f" Max abs diff:   {max_diff:.6f}")
print(f" Mean abs diff:  {mean_diff:.6f}")

# Optional: print few values
print("\nTorchScript output (first 5):", ts_output[0][:5])
print("ONNXRuntime output (first 5):", ort_output[0][:5])

if max_diff < 1e-3:
    print("\n✅ ONNX model matches TorchScript output. Export is valid.")
else:
    print("\n⚠️ Warning: ONNX output differs from TorchScript. Check model export.")