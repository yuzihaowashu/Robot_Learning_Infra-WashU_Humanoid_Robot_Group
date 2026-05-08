# Success Diffusion Policy

Recorded: 2026-05-08 15:06:32 CDT

This document records the first successful real-robot test state for the G1
bottle Diffusion Policy EEF checkpoint.

## Successful Setup

- Robot: Unitree G1
- Task: bottle in/out manipulation
- Model type: Diffusion Policy, EEF version
- Checkpoint:

```text
/home/humanoid-pc/dp_runtime/models/g1-bottle-dp/checkpoints/g1_bottle_in_dp_eef8_epoch100_no_optimizer.ckpt
```

- Policy I/O:
  - observation: `observation.eef.left` 7D + `observation.gripper.left` 1D
  - action: `action.eef.left` 7D + `action.gripper.left` 1D
  - action semantic: absolute EEF pose plus gripper scalar
  - gripper scalar: `0=open`, `1=closed`

## Terminal 1: EEF DP Server

```bash
cd /home/humanoid-pc/yu.zihao/Robot_Learning_Infra-WashU_Humanoid_Robot_Group

bash run_dp.sh server \
  --checkpoint /home/humanoid-pc/dp_runtime/models/g1-bottle-dp/checkpoints/g1_bottle_in_dp_eef8_epoch100_no_optimizer.ckpt \
  --device cuda:0
```

Expected server message:

```text
[DP] Ready. use_ema=True, obs_horizon=2, device=cuda:0, state_dim=8, action_dim=8
```

## Terminal 2: Successful Real Robot Client

This was the successful configuration after the arm stopped moving upward and
the hand was forced to close for the grasp test.

```bash
cd /home/humanoid-pc/yu.zihao/Robot_Learning_Infra-WashU_Humanoid_Robot_Group

bash run_dp.sh client \
  --policy-output eef \
  --hand-mode closed \
  --waist-mode current \
  --max-delta 0.05 \
  --max-hand-delta 0.6 \
  --step-seconds 0.5 \
  --action-exec-steps 4 \
  --server-url http://localhost:8020/act \
  --execute \
  --send-hands \
  --ui
```

Open the UI:

```text
http://localhost:8030
```

Recommended operator flow:

1. Confirm robot network is connected on the `192.168.123.x` subnet.
2. Start the EEF DP server in Terminal 1.
3. Start the real robot UI client in Terminal 2.
4. Open `http://localhost:8030`.
5. Click `Preparation`.
6. Click `Step` only. Do not start with `Run 20`.
7. Watch the terminal output for EEF chunk summary and IK residual.

## Important Conclusions

- The EEF checkpoint is the correct model for this successful test.
- The model output is absolute EEF pose, not delta.
- `--action-mode delta` is not needed for EEF mode and should not be used to
  reason about this checkpoint.
- `--waist-mode current` worked better than forcing waist upright.
- `--max-delta 0.05` made the arm motion safer and reduced sudden jumps.
- `--action-exec-steps 4` used a real action chunk instead of repeatedly
  sampling and executing only the first action.
- The policy gripper scalar stayed near open in the observed tests, so
  `--hand-mode closed` was used to verify grasp behavior independently from
  the learned gripper output.

## Debug Signals To Watch

The client now prints:

```text
EEF chunk summary:
  gripper scalars: [...]
  xyz first -> last: [...] -> [...]
  max xyz step in chunk: ... m; executing first N/8
```

For safe execution:

- `target wrist pos` should not jump behind/up into the robot body.
- `IK pose residual (log6)` should stay small.
- `large arm delta clipped` is acceptable during cautious testing, but if it
  appears every step with the wrong direction, stop and reset.
- If testing learned gripper control, inspect `gripper scalars`; values below
  the close threshold will not close the hand in binary mode.

## Known Caveat

This success used forced closed-hand mode:

```bash
--hand-mode closed
```

The learned gripper scalar still needs separate evaluation. For learned gripper
testing, switch to:

```bash
--hand-mode policy --eef-gripper-mode binary --eef-gripper-open-threshold 0.15 --eef-gripper-close-threshold 0.30
```

or use:

```bash
--hand-mode policy --eef-gripper-mode interp
```

## Smoother Follow-Up Test

After the forced-close success, two small fixes were added:

- EEF IK now seeds each chunk step from the previous commanded arm target, which
  makes a multi-step chunk more continuous.
- Arm execution uses smoothstep interpolation instead of plain linear
  interpolation.
- EEF policy gripper can use smooth float interpolation with low-pass
  smoothing, instead of binary open/close or forced closed mode.

Use this for a smoother release-capable test:

```bash
bash run_dp.sh client \
  --policy-output eef \
  --hand-mode policy \
  --eef-gripper-mode interp \
  --eef-gripper-smoothing 0.5 \
  --waist-mode current \
  --max-delta 0.08 \
  --max-hand-delta 0.6 \
  --step-seconds 0.25 \
  --action-exec-steps 4 \
  --server-url http://localhost:8020/act \
  --execute \
  --send-hands \
  --ui
```

If the arm is still too step-like, increase `--step-seconds` to `0.35`. If it
is too aggressive, reduce `--max-delta` back to `0.05`.
