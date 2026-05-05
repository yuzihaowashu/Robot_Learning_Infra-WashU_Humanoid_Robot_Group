#!/usr/bin/env python3
"""HTTP server for the G1 bottle Diffusion Policy checkpoint."""

from __future__ import annotations

import argparse
import base64
import importlib
import io
import sys
from collections import deque
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import torch
import uvicorn
from fastapi import FastAPI
from fastapi.responses import JSONResponse
from PIL import Image


ROOT_DIR = Path(__file__).resolve().parents[1]
DP_DIR = ROOT_DIR / "Diffusion-Policy"
if str(DP_DIR) not in sys.path:
    sys.path.insert(0, str(DP_DIR))

STATE_INDICES = np.array(
    [
        0, 1, 2, 3, 4, 5, 6,
        15, 16, 17, 18, 19, 20, 21, 22,
        23, 24, 25, 26, 27, 28, 29, 30,
    ],
    dtype=np.int64,
)
ACTION_INDICES = np.array(
    [0, 1, 2, 3, 4, 5, 6, 14, 15, 16, 17, 18, 19, 20],
    dtype=np.int64,
)
DEFAULT_RUNTIME_DIR = Path.home() / "dp_runtime"
DEFAULT_MODEL_DIR = DEFAULT_RUNTIME_DIR / "models" / "g1-bottle-dp"


def expand_action_14_to_31(action_14: np.ndarray) -> np.ndarray:
    action_31 = np.zeros(action_14.shape[:-1] + (31,), dtype=action_14.dtype)
    action_31[..., ACTION_INDICES] = action_14
    return action_31


def decode_image(payload: dict[str, Any]) -> np.ndarray:
    if "image_b64" in payload:
        raw = base64.b64decode(payload["image_b64"])
        image = Image.open(io.BytesIO(raw)).convert("RGB")
        return np.asarray(image)
    if "image" in payload:
        return np.asarray(payload["image"], dtype=np.uint8)
    raise ValueError("Request must include image_b64 or image")


def preprocess_image(image: np.ndarray, image_size: int) -> torch.Tensor:
    if image.ndim != 3 or image.shape[2] != 3:
        raise ValueError(f"Expected RGB image HxWx3, got shape={image.shape}")
    resized = cv2.resize(
        image, (image_size, image_size), interpolation=cv2.INTER_AREA
    )
    chw = np.transpose(resized, (2, 0, 1)).astype(np.float32) / 255.0
    return torch.from_numpy(chw)


def build_state_23(state_63: np.ndarray) -> np.ndarray:
    state_63 = np.asarray(state_63, dtype=np.float32)
    if state_63.shape[-1] != 63:
        raise ValueError(f"Expected state dim 63, got {state_63.shape}")
    return state_63[..., STATE_INDICES]


class DPPolicyServer:
    def __init__(
        self,
        checkpoint: Path,
        device: str,
        image_size: int,
        obs_horizon: int,
    ):
        module = importlib.import_module(
            "diffusion_policy.workspace.train_diffusion_unet_image_workspace"
        )
        workspace_cls = module.TrainDiffusionUnetImageWorkspace

        self.device = torch.device(device)
        self.image_size = image_size
        self.obs_horizon = obs_horizon
        print(f"[DP] Loading checkpoint: {checkpoint}")
        workspace = workspace_cls.create_from_checkpoint(str(checkpoint))
        use_ema = bool(getattr(workspace.cfg.training, "use_ema", False))
        self.policy = workspace.ema_model if use_ema else workspace.model
        self.policy.to(self.device)
        self.policy.eval()
        self.image_history: deque[torch.Tensor] = deque(maxlen=obs_horizon)
        self.state_history: deque[np.ndarray] = deque(maxlen=obs_horizon)
        print(
            "[DP] Ready. "
            f"use_ema={use_ema}, obs_horizon={obs_horizon}, "
            f"device={self.device}"
        )

    def reset(self) -> dict[str, Any]:
        self.image_history.clear()
        self.state_history.clear()
        return {
            "status": "ok",
            "message": (
                "observation history reset; next inference starts fresh"
            ),
        }

    def predict(self, payload: dict[str, Any]) -> dict[str, Any]:
        image = preprocess_image(decode_image(payload), self.image_size)
        state_23 = build_state_23(
            np.asarray(payload["state"], dtype=np.float32)
        )

        self.image_history.append(image)
        self.state_history.append(state_23)
        while len(self.image_history) < self.obs_horizon:
            self.image_history.appendleft(image.clone())
            self.state_history.appendleft(state_23.copy())

        color = torch.stack(list(self.image_history), dim=0)
        states = np.stack(list(self.state_history), axis=0).astype(np.float32)
        obs = {
            "color_0": color.unsqueeze(0).to(self.device),
            "states": torch.from_numpy(states).unsqueeze(0).to(self.device),
        }

        with torch.inference_mode():
            result = self.policy.predict_action(obs)
        action_14 = result["action"].detach().cpu().numpy().astype(np.float32)
        action_31 = expand_action_14_to_31(action_14)

        return {
            "action_14": action_14[0].tolist(),
            "action_31": action_31[0].tolist(),
            "action_indices": ACTION_INDICES.tolist(),
            "state_indices": STATE_INDICES.tolist(),
            "text_conditioned": False,
            "task": "g1-bottle-in-out",
        }


def default_checkpoint(model_dir: Path) -> Path:
    return model_dir / "checkpoints" / "g1_bottle_dp_14act_epoch25.ckpt"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--checkpoint", type=Path, default=None)
    parser.add_argument("--model-dir", type=Path, default=DEFAULT_MODEL_DIR)
    parser.add_argument("--host", type=str, default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8020)
    parser.add_argument("--device", type=str, default="cuda:0")
    parser.add_argument("--image-size", type=int, default=84)
    parser.add_argument("--obs-horizon", type=int, default=2)
    parser.add_argument(
        "--load-only",
        action="store_true",
        help="Load the checkpoint and exit without starting HTTP service.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    checkpoint = args.checkpoint or default_checkpoint(args.model_dir)
    if not checkpoint.exists():
        raise FileNotFoundError(
            f"Checkpoint not found: {checkpoint}\n"
            "Run: bash run_dp.sh download"
        )

    server = DPPolicyServer(
        checkpoint=checkpoint,
        device=args.device,
        image_size=args.image_size,
        obs_horizon=args.obs_horizon,
    )
    if args.load_only:
        print("[DP] Checkpoint load-only validation succeeded.")
        return
    app = FastAPI()

    @app.get("/health")
    def health():
        return {"status": "ok"}

    @app.get("/reset")
    def reset():
        return server.reset()

    @app.post("/act")
    def act(payload: dict[str, Any]):
        try:
            return JSONResponse(content=server.predict(payload))
        except Exception as exc:
            return JSONResponse(content={"error": str(exc)}, status_code=500)

    uvicorn.run(app, host=args.host, port=args.port)


if __name__ == "__main__":
    main()
