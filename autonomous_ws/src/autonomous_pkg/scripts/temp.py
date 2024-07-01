import torch

A = torch.tensor([[1,1,1],[2,2,2],[3,3,3]])
B = torch.tensor([1,1,1])

print(torch.sqrt(torch.sum((A -B)** 2, dim=1)))