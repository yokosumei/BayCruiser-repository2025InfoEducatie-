import torch

ckpt = torch.load("my_model.pt", map_location="cpu")
print(ckpt.get("train_args", {}).get("version", "versiune necunoscută"))
