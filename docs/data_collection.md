# Data Collection — LeRobot Dataset from Taught Trajectories

Collect training datasets by replaying drag-and-teach trajectories on the
real robot while recording joint states, hand positions, and camera images
into the LeRobot format (Parquet + MP4).

## Workflow

```
Step 1: Record trajectories by dragging the robot's arms
    bash run_teach.sh record
    → trajectories/traj_20260306_174754.json

Step 2: Replay + Record into LeRobot dataset
    bash run_collect.sh --task "pick up apple" trajectories/traj_*.json
    → datasets/yuzihaowashu/g1_pick_apple/

Step 3: Visualize locally
    python -m lerobot.scripts.lerobot_dataset_viz \
        --repo-id yuzihaowashu/g1_pick_apple \
        --root ./datasets/yuzihaowashu/g1_pick_apple

Step 4: Push to HuggingFace Hub (optional)
    Add --push flag, or push later manually
```

## Usage

```bash
# Collect from all recorded trajectories
bash run_collect.sh \
    --task "pick up the apple and place on plate" \
    trajectories/traj_*.json

# Collect specific files with custom repo ID
bash run_collect.sh \
    --task "wave hello" \
    --repo-id yuzihaowashu/g1_wave \
    trajectories/traj_001.json trajectories/traj_002.json

# Collect and immediately push to Hub
bash run_collect.sh \
    --task "pick up apple" \
    --push \
    trajectories/traj_*.json
```

### Options

| Flag | Default | Description |
|------|---------|-------------|
| `--task TEXT` | (required) | Task description for all episodes |
| `--repo-id ID` | `yuzihaowashu/g1_demonstrations` | HuggingFace dataset ID |
| `--output-dir DIR` | `./datasets/` | Local storage directory |
| `--push` | off | Push to HuggingFace Hub after collection |

## What Gets Recorded

During each trajectory replay, every frame (30 Hz) captures:

| Feature | Shape | Description |
|---------|-------|-------------|
| `observation.state` | (29,) | All body joint positions (radians) |
| `observation.hand` | (14,) | Hand motor positions (7 left + 7 right) |
| `observation.images.ego_view` | (H, W, 3) | Head camera RGB frame |
| `action` | (17,) | Commanded arm+waist positions (3 waist + 14 arm) |
| `action.hand` | (14,) | Commanded hand positions |

## Dataset Format

The output follows LeRobot v2 format:

```
datasets/yuzihaowashu/g1_pick_apple/
├── meta/
│   ├── info.json           # Dataset config (fps, robot_type, features)
│   ├── episodes.jsonl      # Episode metadata (task, length)
│   ├── tasks.jsonl         # Task descriptions
│   └── stats.json          # Normalization statistics
├── data/
│   └── chunk-000/
│       ├── episode_000000.parquet   # Joint states + actions
│       ├── episode_000001.parquet
│       └── ...
└── videos/
    └── chunk-000/
        └── observation.images.ego_view/
            ├── episode_000000.mp4   # Camera recordings
            ├── episode_000001.mp4
            └── ...
```

## Collection Process

For each trajectory file:

1. **Move to start** — Smoothly interpolates arms to the trajectory's
   starting position (3 seconds, smoothstep)

2. **Replay + Record** — Replays the trajectory at 30 Hz while
   capturing observation, action, and camera data every frame

3. **Return home** — After the episode, smoothly returns to the
   initial home pose

4. **User approval** — Before each episode, waits for Enter key.
   Press Ctrl+C to stop early (data collected so far is saved).

## Tips for Good Data

- **Repeat the same task 50-100 times** with slight variations
  (different object positions, approach angles)
- **Keep the camera view consistent** — the model learns from ego_view
- **Use clear task descriptions** — they become the language conditioning
- **Check quality** — visualize after collection to verify alignment

## Using with GR00T

The LeRobot dataset can be converted for GR00T fine-tuning:

1. Convert v3 → v2 if needed (our output is already v2-compatible)
2. Add `meta/modality.json` mapping state/action indices
3. Generate statistics: `python gr00t/data/stats.py <path> UNITREE_G1`
4. Fine-tune: `python gr00t/experiment/launch_finetune.py ...`

See [VLA Inference docs](vla_inference.md) for the full GR00T pipeline.

## Implementation

- `utils/collect_dataset.py` — Main collection script
- `run_collect.sh` — Entry point with argument parsing
