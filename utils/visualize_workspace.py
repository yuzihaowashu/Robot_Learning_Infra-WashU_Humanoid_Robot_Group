"""
Visualize G1 arm workspace envelope from URDF joint limits.

Generates a rotating 3D GIF showing:
- Left arm workspace (blue)
- Right arm workspace (red)
- Robot body outline
- Joint limit boundaries

Uses Pinocchio for FK and matplotlib for rendering.

Usage:
    conda activate lerobot
    python visualize_workspace.py
"""

import numpy as np
import pinocchio as pin
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

URDF_PATH = "/home/humanoid-pc/unitree_rl_gym/resources/robots/g1_description/g1_29dof_with_hand_rev_1_0.urdf"

LEFT_PALM_FRAME = "left_hand_palm_link"
RIGHT_PALM_FRAME = "right_hand_palm_link"

# q-vector indices in the URDF Pinocchio model
# Legs: 0-11, Waist: 12-14, Left arm: 15-21, Left hand: 22-28,
# Right arm: 29-35, Right hand: 36-42
LEFT_ARM_Q_INDICES = list(range(15, 22))
RIGHT_ARM_Q_INDICES = list(range(29, 36))

N_SAMPLES = 40000
OUTPUT_PATH = "/home/humanoid-pc/chongjie.zhang/docs/g1_arm_workspace.gif"

# Simplified body collision volumes (cylinders/boxes centered at origin)
# Each entry: (center_x, center_y, center_z, half_x, half_y, half_z)
BODY_BOXES = [
    (0.0, 0.0, 0.20, 0.14, 0.14, 0.22),   # torso
    (0.0, 0.0, -0.05, 0.10, 0.10, 0.05),   # hip area
    (0.0, 0.0, 0.45, 0.10, 0.08, 0.06),    # upper chest / neck
]


def is_inside_body(point):
    """Check if a 3D point is inside any of the simplified body volumes."""
    x, y, z = point
    for cx, cy, cz, hx, hy, hz in BODY_BOXES:
        if (abs(x - cx) < hx and abs(y - cy) < hy and abs(z - cz) < hz):
            return True
    return False


def sample_workspace(model, data, arm_joints, ee_frame_id, n_samples):
    """Sample end-effector positions by randomizing arm joints within limits."""
    positions = []
    q = pin.neutral(model)

    lower = model.lowerPositionLimit.copy()
    upper = model.upperPositionLimit.copy()

    for _ in range(n_samples):
        for j in arm_joints:
            q[j] = np.random.uniform(lower[j], upper[j])

        pin.forwardKinematics(model, data, q)
        pin.updateFramePlacements(model, data)
        pos = data.oMf[ee_frame_id].translation.copy()
        positions.append(pos)

    return np.array(positions)


def get_arm_chain_positions(model, data, arm_joints, q):
    """Get 3D positions of each joint in the arm kinematic chain for a given config."""
    pin.forwardKinematics(model, data, q)
    pin.updateFramePlacements(model, data)

    chain_pos = []
    for j in arm_joints:
        joint_name = model.names[j + 1]  # pinocchio joint indices are 1-based
        frame_id = model.getFrameId(joint_name)
        if frame_id < model.nframes:
            chain_pos.append(data.oFrame[frame_id].translation.copy())
    return np.array(chain_pos) if chain_pos else np.zeros((0, 3))


def draw_body_outline(ax):
    """Draw the body collision volumes used for filtering."""
    for cx, cy, cz, hx, hy, hz in BODY_BOXES:
        verts = [
            [cx - hx, cy - hy, cz - hz], [cx + hx, cy - hy, cz - hz],
            [cx + hx, cy + hy, cz - hz], [cx - hx, cy + hy, cz - hz],
            [cx - hx, cy - hy, cz + hz], [cx + hx, cy - hy, cz + hz],
            [cx + hx, cy + hy, cz + hz], [cx - hx, cy + hy, cz + hz],
        ]
        faces = [
            [verts[j] for j in [0, 1, 5, 4]],
            [verts[j] for j in [2, 3, 7, 6]],
            [verts[j] for j in [0, 3, 7, 4]],
            [verts[j] for j in [1, 2, 6, 5]],
            [verts[j] for j in [4, 5, 6, 7]],
            [verts[j] for j in [0, 1, 2, 3]],
        ]
        poly = Poly3DCollection(faces, alpha=0.25, facecolor="orange", edgecolor="darkorange", linewidth=0.5)
        ax.add_collection3d(poly)


def main():
    print("Loading URDF model...")
    model = pin.buildModelFromUrdf(URDF_PATH)
    data = model.createData()

    left_frame_id = model.getFrameId(LEFT_PALM_FRAME)
    right_frame_id = model.getFrameId(RIGHT_PALM_FRAME)

    print(f"Sampling {N_SAMPLES} configurations per arm...")
    left_ws_raw = sample_workspace(model, data, LEFT_ARM_Q_INDICES, left_frame_id, N_SAMPLES)
    right_ws_raw = sample_workspace(model, data, RIGHT_ARM_Q_INDICES, right_frame_id, N_SAMPLES)

    # Filter out points inside the body
    left_mask = np.array([not is_inside_body(p) for p in left_ws_raw])
    right_mask = np.array([not is_inside_body(p) for p in right_ws_raw])
    left_ws = left_ws_raw[left_mask]
    right_ws = right_ws_raw[right_mask]
    print(f"After body collision filter: left {left_mask.sum()}/{N_SAMPLES}, "
          f"right {right_mask.sum()}/{N_SAMPLES}")

    # Also compute a few arm chain poses for visualization
    q_neutral = pin.neutral(model)
    chain_configs = []
    for _ in range(8):
        q = q_neutral.copy()
        lower = model.lowerPositionLimit
        upper = model.upperPositionLimit
        for j in LEFT_ARM_Q_INDICES + RIGHT_ARM_Q_INDICES:
            q[j] = np.random.uniform(lower[j], upper[j])
        chain_configs.append(q.copy())

    print("Rendering GIF...")
    fig = plt.figure(figsize=(12, 8))
    ax = fig.add_subplot(111, projection="3d")

    def update(frame_num):
        ax.cla()

        # Workspace point clouds
        ax.scatter(left_ws[:, 0], left_ws[:, 1], left_ws[:, 2],
                   c="dodgerblue", s=0.3, alpha=0.15, label="Left arm workspace")
        ax.scatter(right_ws[:, 0], right_ws[:, 1], right_ws[:, 2],
                   c="tomato", s=0.3, alpha=0.15, label="Right arm workspace")

        # Draw sample arm chains
        for q in chain_configs:
            pin.forwardKinematics(model, data, q)
            pin.updateFramePlacements(model, data)

            left_joint_ids = [16, 17, 18, 19, 20, 21, 22]   # Pinocchio joint IDs
            right_joint_ids = [30, 31, 32, 33, 34, 35, 36]
            for joint_ids, color in [(left_joint_ids, "blue"), (right_joint_ids, "red")]:
                joint_frames = []
                for jid in joint_ids:
                    jname = model.names[jid]
                    fid = model.getFrameId(jname)
                    if fid < model.nframes:
                        joint_frames.append(data.oMf[fid].translation.copy())
                if joint_frames:
                    pts = np.array(joint_frames)
                    ax.plot(pts[:, 0], pts[:, 1], pts[:, 2],
                            color=color, alpha=0.3, linewidth=1.0)

        draw_body_outline(ax)

        # Shoulder joint origins (approximate)
        ax.scatter([0], [0], [0.35], c="black", s=50, marker="^", zorder=5)

        ax.set_xlabel("X (forward)", fontsize=9)
        ax.set_ylabel("Y (left)", fontsize=9)
        ax.set_zlabel("Z (up)", fontsize=9)
        ax.set_title("Unitree G1 — Arm Workspace Envelope\n"
                      "(URDF joint limits, body collision filtered)",
                      fontsize=11, fontweight="bold")

        lim = 0.8
        ax.set_xlim(-lim, lim)
        ax.set_ylim(-lim, lim)
        ax.set_zlim(-0.4, 1.0)

        ax.legend(loc="upper left", fontsize=8, markerscale=8)

        # Rotate view
        ax.view_init(elev=20, azim=frame_num * 3)

    n_frames = 120  # 360 degrees / 3 per frame
    anim = FuncAnimation(fig, update, frames=n_frames, interval=50)
    anim.save(OUTPUT_PATH, writer=PillowWriter(fps=20))
    plt.close()

    print(f"\nSaved to: {OUTPUT_PATH}")
    print(f"Left arm reach: X=[{left_ws[:,0].min():.2f}, {left_ws[:,0].max():.2f}] "
          f"Y=[{left_ws[:,1].min():.2f}, {left_ws[:,1].max():.2f}] "
          f"Z=[{left_ws[:,2].min():.2f}, {left_ws[:,2].max():.2f}]")
    print(f"Right arm reach: X=[{right_ws[:,0].min():.2f}, {right_ws[:,0].max():.2f}] "
          f"Y=[{right_ws[:,1].min():.2f}, {right_ws[:,1].max():.2f}] "
          f"Z=[{right_ws[:,2].min():.2f}, {right_ws[:,2].max():.2f}]")


if __name__ == "__main__":
    main()
