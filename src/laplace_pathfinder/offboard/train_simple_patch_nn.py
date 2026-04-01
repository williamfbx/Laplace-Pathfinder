#!/usr/bin/env python3
"""Train a simple patch NN and export TorchScript for perturbation_node.cpp.

Input channels (N, 3, H, W):
- ch0: initial patch phi
- ch1: fixed-mask (1 for fixed/perimeter/obstacle)
- ch2: wall-mask  (1 for obstacle)

Output:
- predicted perturbation delta-phi (N, 1, H, W)
"""

import argparse
import os
import random
from typing import Tuple

import torch
import torch.nn as nn
import torch.optim as optim


class SimplePatchCNN(nn.Module):
    def __init__(self, in_ch: int = 3, base: int = 32):
        super().__init__()
        self.net = nn.Sequential(
            nn.Conv2d(in_ch, base, kernel_size=3, padding=1),
            nn.ReLU(inplace=True),
            nn.Conv2d(base, base, kernel_size=3, padding=1),
            nn.ReLU(inplace=True),
            nn.Conv2d(base, base, kernel_size=3, padding=1),
            nn.ReLU(inplace=True),
            nn.Conv2d(base, 1, kernel_size=1),
        )

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        return self.net(x)


def solve_patch_jacobi(
    phi_init: torch.Tensor,
    fixed_mask: torch.Tensor,
    wall_mask: torch.Tensor,
    iters: int = 300,
) -> torch.Tensor:
    """Simple Jacobi solver to generate pseudo-ground-truth labels."""
    phi = phi_init.clone()
    free_mask = (~fixed_mask.bool()).float()

    for _ in range(iters):
        up = torch.roll(phi, shifts=1, dims=0)
        down = torch.roll(phi, shifts=-1, dims=0)
        left = torch.roll(phi, shifts=1, dims=1)
        right = torch.roll(phi, shifts=-1, dims=1)

        avg = 0.25 * (up + down + left + right)
        phi = free_mask * avg + (1.0 - free_mask) * phi

        # Keep obstacle cells pinned to zero.
        phi = torch.where(wall_mask.bool(), torch.zeros_like(phi), phi)

    return phi


def make_sample(h: int, w: int, device: torch.device) -> Tuple[torch.Tensor, torch.Tensor]:
    fixed = torch.zeros((h, w), dtype=torch.float32, device=device)
    wall = torch.zeros((h, w), dtype=torch.float32, device=device)

    # Perimeter is fixed.
    fixed[0, :] = 1.0
    fixed[-1, :] = 1.0
    fixed[:, 0] = 1.0
    fixed[:, -1] = 1.0

    # Random obstacle rectangles in interior.
    num_obs = random.randint(1, 5)
    for _ in range(num_obs):
        rh = random.randint(2, max(2, h // 6))
        rw = random.randint(2, max(2, w // 6))
        r0 = random.randint(1, max(1, h - rh - 2))
        c0 = random.randint(1, max(1, w - rw - 2))
        wall[r0 : r0 + rh, c0 : c0 + rw] = 1.0

    fixed = torch.clamp(fixed + wall, 0.0, 1.0)

    # Random boundary values, interior starts at 0.
    phi0 = torch.zeros((h, w), dtype=torch.float32, device=device)
    phi0[0, :] = torch.rand((w,), device=device)
    phi0[-1, :] = torch.rand((w,), device=device)
    phi0[:, 0] = torch.rand((h,), device=device)
    phi0[:, -1] = torch.rand((h,), device=device)

    # Obstacles pinned to 0.
    phi0 = torch.where(wall.bool(), torch.zeros_like(phi0), phi0)

    target_phi = solve_patch_jacobi(phi0, fixed, wall, iters=350)
    target_delta = target_phi - phi0
    # By construction, perturbation is zero on all fixed cells.
    target_delta = target_delta * (1.0 - fixed)

    x = torch.stack([phi0, fixed, wall], dim=0)      # (3, H, W)
    y = target_delta.unsqueeze(0)                     # (1, H, W)
    return x, y


def make_batch(batch_size: int, h: int, w: int, device: torch.device) -> Tuple[torch.Tensor, torch.Tensor]:
    xs, ys = [], []
    for _ in range(batch_size):
        x, y = make_sample(h, w, device)
        xs.append(x)
        ys.append(y)
    return torch.stack(xs, dim=0), torch.stack(ys, dim=0)


def train(args: argparse.Namespace) -> None:
    device = torch.device("cuda" if args.device == "cuda" and torch.cuda.is_available() else "cpu")
    model = SimplePatchCNN(in_ch=3, base=args.base_channels).to(device)
    opt = optim.Adam(model.parameters(), lr=args.lr)
    loss_fn = nn.MSELoss()

    print(f"Training on {device}, patch_size={args.patch_h}x{args.patch_w}")

    model.train()
    for epoch in range(1, args.epochs + 1):
        x, y = make_batch(args.batch_size, args.patch_h, args.patch_w, device)
        pred = model(x)

        # Enforce zero perturbation on fixed cells for stable learning.
        fixed = x[:, 1:2, :, :]
        pred = pred * (1.0 - fixed)

        loss = loss_fn(pred, y)
        opt.zero_grad()
        loss.backward()
        opt.step()

        if epoch % args.log_every == 0 or epoch == 1:
            print(f"epoch={epoch:4d}  loss={loss.item():.6f}")

    os.makedirs(os.path.dirname(args.output), exist_ok=True)
    model.eval().cpu()

    # Scripted module expected by C++ node: input (1,3,H,W), output (1,1,H,W).
    scripted = torch.jit.script(model)
    scripted.save(args.output)
    print(f"Saved TorchScript model to: {args.output}")


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser()
    p.add_argument("--output", type=str, default="src/laplace_pathfinder/models/patch_nn_scripted.pt")
    p.add_argument("--device", type=str, default="cuda", choices=["cpu", "cuda"])
    p.add_argument("--epochs", type=int, default=800)
    p.add_argument("--batch-size", type=int, default=32)
    p.add_argument("--lr", type=float, default=1e-3)
    p.add_argument("--patch-h", type=int, default=64)
    p.add_argument("--patch-w", type=int, default=64)
    p.add_argument("--base-channels", type=int, default=32)
    p.add_argument("--log-every", type=int, default=50)
    return p.parse_args()


if __name__ == "__main__":
    random.seed(0)
    torch.manual_seed(0)
    train(parse_args())
