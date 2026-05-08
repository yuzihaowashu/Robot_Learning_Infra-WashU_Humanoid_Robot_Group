# G1 Bottle Dimension Alignment

This document records how the G1 bottle data, Diffusion Policy deployment, and
Psi0 deployment interpret robot state/action dimensions.

Short answer:

```text
Diffusion Policy 14D mapping: fixed.
Psi0 31D deployment decode: fixed to match the HEPosttrain checkpoint layout.
```

The core issue is not whether Psi0 trains on 31D action instead of 14D action.
Training on the full 31D vector is acceptable if inactive joints stay zero or are
ignored at execution time. The dangerous issue is whether dimension `i` means
the same physical joint during training and deployment.

## Canonical Data Layout

XR teleoperation records structured fields:

```text
states.left_arm     7D
states.right_arm    7D
states.left_ee      7D
states.right_ee     7D
states.body         task/mode dependent

actions.left_arm    7D
actions.right_arm   7D
actions.left_ee     7D
actions.right_ee    7D
actions.body        task/mode dependent
```

For this task, `left_ee` and `right_ee` are the Dex3 hands. In left-only
teleoperation, the active side is the left arm plus left hand, but the right side
is still present in the recorded schema so downstream data shapes stay stable.

`utils/xr_to_lerobot.py` flattens both state and action in this order:

```python
def build_action(frame):
    return (
        actions.left_arm
        + actions.right_arm
        + actions.left_ee
        + actions.right_ee
        + actions.body
    )
```

Therefore the LeRobot-style 31D action layout is:

```text
0..6      left_arm[0:7]
7..13     right_arm[0:7]
14..20    left_hand[0:7]
21..27    right_hand[0:7]
28..30    body / torso slots
```

This is the layout used by `utils/xr_to_lerobot.py`. It is not the layout used
by the current Psi0 10k/40k checkpoints, which were trained through Psi0's
`HEPosttrainRepackTransform` path.

## Diffusion Policy Status

Diffusion Policy should use a semantic 14D left-side interface:

```text
policy action[0:7]    -> left_arm[0:7]
policy action[7:14]   -> left_hand[0:7]
```

When expanding the 14D policy output into the 31D deployment layout, use:

```python
ACTION_INDICES = [
    0, 1, 2, 3, 4, 5, 6,       # left_arm[0:7]
    14, 15, 16, 17, 18, 19, 20 # left_hand[0:7]
]
```

This has been fixed in the local DP config and deployment code:

```text
Diffusion-Policy task configs:
  diffusion_policy/config/task/g1_bottle_image.yaml
  diffusion_policy/config/task/g1_bottle_in_image.yaml

Robot_Learning_Infra DP deployment:
  utils/dp_policy_server.py
  utils/dp_g1_client.py
  run_dp.sh validate
```

The old mapping must not be used for new DP training:

```python
# Do not use this for new DP runs.
[0, 1, 2, 3, 4, 5, 6, 15, 16, 17, 18, 19, 20, 30]
```

That old mapping came from observed non-zero dimensions, but it skips
`left_hand[0]` and incorrectly uses body slot `30` as the 14th policy action.

Important consequence: any checkpoint trained with the old mapping should be
treated as old-layout. Use the corrected mapping for a new DP training run.

## Psi0 Training Status

The current G1 bottle Psi0 Slurm jobs use the full dataset shapes:

```text
STATE_DIM=63
ACTION_DIM=31
ACTION_CHUNK_SIZE=30
ACTION_EXEC_HORIZON=30
```

The training command passes those dimensions into both the data transforms and
the model:

```text
--data.transform.repack.pad-action-dim=31
--data.transform.repack.pad-state-dim=63
--data.transform.field.pad-action-dim=31
--data.transform.field.pad-state-dim=63
--model.action-dim=31
--model.odim=63
--model.action-chunk-size=30
--model.action-exec-horizon=30
```

The actual 10k/40k run config parses to `HEPosttrainRepackTransform`, not
`RealRepackTransform`. For G1, `raw_he_to_psi0.py` writes
`action.joint_angles` as:

```python
action = hand_joints + arm_joints
```

Then `HEPosttrainRepackTransform` keeps that order for G1 and constructs state
as:

```python
states = observation.hand_joints + observation.arm_joints
```

So the current Psi0 training should be interpreted as:

```text
input state:   full 63D, with first 28D = hands then arms
output action: full 31D, with first 28D = hands then arms
left-only:     only implied by the data distribution and deployment choice,
               not by a true 14D model head
```

This is not automatically wrong. It can be a reasonable first version if the
deployment client decodes the 31D output using the same order as the dataset.

## Psi0 Deployment Status

The local Psi0 RTC client now decodes the model output in
`utils/psi0_rtc_bimanual_client.py` using the same hand-first order as the
HEPosttrain G1 checkpoint:

```python
hand = action[:14].copy()
arm = action[14:28].copy()
```

That means deployment interprets the first 28 dimensions as:

```text
0..6      left_hand[0:7]
7..13     right_hand[0:7]
14..20    left_arm[0:7]
21..27    right_arm[0:7]
28..30    ignored body / torso slots
```

With `--arm-side left`, the client then executes:

```text
left arm:  action[14:21]
left hand: action[0:7]
right arm: held at previous target
right hand: opened
```

This matches the HEPosttrain checkpoint layout for the arm/hand blocks. The body
slots remain ignored by the RTC client.

## The Dangerous Mismatch

If this Psi0 checkpoint is decoded as the generic arm-first LeRobot layout, the
client would read:

```text
0..6      left_arm
7..13     right_arm
14..20    left_hand
21..27    right_hand
28..30    body
```

But the actual HEPosttrain G1 checkpoint was trained with:

```text
0..6      left_hand
7..13     right_hand
14..20    left_arm
21..27    right_arm
28..30    body ignored
```

Using arm-first decode for this checkpoint would swap arm and hand semantics at
deployment. The model output intended for the left hand would be treated as left
arm command, and the model output intended for the left arm would be treated as
left hand command. That is much more dangerous than having unused right-side or
body dimensions in a 31D output.

This has now been fixed in the RTC client by changing only the deployment
decode. The Psi0 model architecture and training dimensions remain unchanged.

## Did We Fix It?

Yes, for the local Psi0 RTC deployment client.

What has been fixed:

```text
Diffusion Policy 14D train/deploy mapping.
Psi0 RTC 31D action decode order.
```

What remains unchanged:

```text
Psi0 training still uses full 63D state / 31D action.
Psi0 model architecture is unchanged.
Right-side and body dimensions may still appear in the model output, but the
left-only deployment path only executes left arm and left hand.
```

Before running Psi0 on the real robot, still do a dry-run / UI preview check and
verify that the printed left hand target changes correspond to action dimensions
`0..6`, while the left arm target corresponds to dimensions `14..20`.

## Recommended Fix

Do not change the Psi0 model architecture first. Keep:

```text
STATE_DIM=63
ACTION_DIM=31
ACTION_CHUNK_SIZE=30
```

The chosen fix is to decode deployment actions using the training data layout:

```python
left_hand = action[0:7]
right_hand = action[7:14]
left_arm = action[14:21]
right_arm = action[21:28]
body = action[28:31]  # ignored or handled separately
```

For `--arm-side left`, execute:

```text
left arm:  action[14:21]
left hand: action[0:7]
right arm: previous target / hold
right hand: open or current, depending on safety mode
```

Only after this full 63D/31D path is validated should we consider a true Psi0
14D left-only model. A true 14D Psi0 model would require a data transform that
slices the dataset action into `left_arm + left_hand` and an action head with
`ACTION_DIM=14`, which is a larger training/checkpoint-compatibility change.

## Change Record

Reason for the change:

```text
Psi0 10k/40k training uses HEPosttrain G1 action order: hands then arms.
The RTC client briefly decoded model output in arm-first order.
That mismatch could swap hand and arm commands on the robot.
```

Changes made:

```text
utils/psi0_rtc_bimanual_client.py
  Changed decode_policy_action():
    correct: hand = action[:14], arm = action[14:28]

docs/g1_bottle_dimension_alignment.md
  Recorded the data layout, DP fix, Psi0 risk analysis, selected fix, and
  current remaining assumptions.
```
