#!/usr/bin/env python3

import argparse
import glob
import os
import random
from pathlib import Path
from typing import Tuple

import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
from torch.optim.lr_scheduler import StepLR
from torch.utils.data import DataLoader
from torch.utils.data import Dataset


class ResidualBlock(nn.Module):
    def __init__(self, ch: int):
        super().__init__()
        self.conv1 = nn.Conv2d(ch, ch, kernel_size=3, padding=1)
        self.act1 = nn.SiLU(inplace=True)
        self.conv2 = nn.Conv2d(ch, ch, kernel_size=3, padding=1)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        out = self.conv1(x)
        out = self.act1(out)
        out = self.conv2(out)
        return x + out


class SimplePatchCNN(nn.Module):
    def __init__(self, in_ch: int = 3, base: int = 64, blocks: int = 8):
        super().__init__()
        self.stem = nn.Sequential(
            nn.Conv2d(in_ch, base, kernel_size=3, padding=1),
            nn.SiLU(inplace=True),
        )
        self.body = nn.Sequential(*[ResidualBlock(base) for _ in range(blocks)])
        self.head = nn.Sequential(
            nn.SiLU(inplace=True),
            nn.Conv2d(base, base // 2, kernel_size=3, padding=1),
            nn.SiLU(inplace=True),
            nn.Conv2d(base // 2, 1, kernel_size=1),
        )

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        x = self.stem(x)
        x = self.body(x)
        return self.head(x)


class CollectedPatchDataset(Dataset):
    def __init__(self, data_dir: str):
        self.sample_dirs = sorted(glob.glob(str(Path(data_dir) / "sample_*")))
        if not self.sample_dirs:
            raise RuntimeError(f"No samples found under {data_dir}/sample_*")

    def __len__(self) -> int:
        return len(self.sample_dirs)

    def __getitem__(self, idx: int) -> Tuple[torch.Tensor, torch.Tensor]:
        sample_dir = Path(self.sample_dirs[idx])

        patch_phi = np.loadtxt(sample_dir / "patch_phi.csv", delimiter=",", dtype=np.float32)
        patch_fixed = np.loadtxt(sample_dir / "patch_fixed.csv", delimiter=",", dtype=np.float32)
        patch_wall = np.loadtxt(sample_dir / "patch_wall.csv", delimiter=",", dtype=np.float32)
        delta = np.loadtxt(sample_dir / "delta.csv", delimiter=",", dtype=np.float32)

        x = np.stack([patch_phi, patch_fixed, patch_wall], axis=0)
        y = np.expand_dims(delta, axis=0)

        return torch.from_numpy(x), torch.from_numpy(y)


def train(args: argparse.Namespace) -> None:
    device = torch.device("cuda" if args.device == "cuda" and torch.cuda.is_available() else "cpu")
    model = SimplePatchCNN(in_ch=3, base=args.base_channels, blocks=args.res_blocks).to(device)
    opt = optim.Adam(model.parameters(), lr=args.lr)
    scheduler = StepLR(opt, step_size=args.lr_step, gamma=args.lr_gamma)
    dataset = CollectedPatchDataset(args.data_dir)
    loader = DataLoader(dataset, batch_size=args.batch_size, shuffle=True, drop_last=False)

    print(f"Training on {device}, dataset={args.data_dir}, samples={len(dataset)}")

    best_loss = float("inf")
    best_state = None

    model.train()
    for epoch in range(1, args.epochs + 1):
        running_loss = 0.0
        batch_count = 0
        for x, y in loader:
            x = x.to(device)
            y = y.to(device)
            pred = model(x)

            # Enforce zero perturbation on fixed cells
            fixed = x[:, 1:2, :, :]
            free = 1.0 - fixed
            pred = pred * free
            target = y * free

            sq_error = (pred - target) ** 2
            denom = torch.clamp(free.sum(), min=1.0)
            loss = sq_error.sum() / denom
            opt.zero_grad()
            loss.backward()
            opt.step()

            running_loss += float(loss.item())
            batch_count += 1

        epoch_loss = running_loss / max(batch_count, 1)
        if epoch_loss < best_loss:
            best_loss = epoch_loss
            best_state = {k: v.detach().cpu().clone() for k, v in model.state_dict().items()}

        if epoch % args.log_every == 0 or epoch == 1:
            current_lr = opt.param_groups[0]["lr"]
            print(f"epoch={epoch:4d}  loss={epoch_loss:.6f}  best={best_loss:.6f}  lr={current_lr:.2e}")

        scheduler.step()

    if best_state is not None:
        model.load_state_dict(best_state)
        print(f"Restored best model with loss={best_loss:.6f}")

    os.makedirs(os.path.dirname(args.output), exist_ok=True)
    model.eval().cpu()

    # Scripted module expected by C++ node: input (1,3,H,W), output (1,1,H,W).
    scripted = torch.jit.script(model)
    scripted.save(args.output)
    print(f"Saved TorchScript model to: {args.output}")


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser()
    p.add_argument("--output", type=str, default="src/laplace_pathfinder/models/laplace_nn.pt")
    p.add_argument("--device", type=str, default="cuda", choices=["cpu", "cuda"])
    p.add_argument("--epochs", type=int, default=800)
    p.add_argument("--batch-size", type=int, default=32)
    p.add_argument("--lr", type=float, default=1e-3)
    p.add_argument("--data-dir", type=str, default="src/laplace_pathfinder/data/")
    p.add_argument("--patch-h", type=int, default=64)
    p.add_argument("--patch-w", type=int, default=64)
    p.add_argument("--base-channels", type=int, default=64)
    p.add_argument("--res-blocks", type=int, default=8)
    p.add_argument("--lr-step", type=int, default=200)
    p.add_argument("--lr-gamma", type=float, default=0.5)
    p.add_argument("--log-every", type=int, default=25)
    return p.parse_args()


if __name__ == "__main__":
    random.seed(0)
    torch.manual_seed(0)
    train(parse_args())
