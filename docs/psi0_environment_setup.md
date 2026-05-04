# Psi0 Environment Setup (Embedded Copy in WashU Humanoid Repository)

This document explains which tool should be used to manage the Python environment for **Psi0** inside this repository, and the recommended installation order. Upstream Psi0 primarily uses **uv**; the **real-robot deployment** subdirectory has a separate **conda** workflow.

| Purpose | Tool | Typical Environment Name / Path | Notes |
|------|------|-------------------|------|
| Ψ₀ training, inference, baselines, and development aligned with the repository `pyproject.toml` | **uv** | `.venv-psi/` at the Psi0 repository root | Recommended by the official README; consistent with `uv sync` / `uv pip` |
| Real-robot data collection / RTC deployment (`Psi0/real/`) | **conda** | `psi_deploy` (created from YAML) | IK, Unitree SDK, camera-side dependencies, etc.; see `Psi0/real/README.md` |
| Optional: full Nix development shell (including SIMPLE, etc.) | **nix** | `nix develop` | See `Psi0/examples/quick_start/psi.md`; not required |

For authoritative details, always refer to **`Psi0/README.md`** and **`Psi0/real/README.md`**. This document is only a decision summary for "uv or conda?" plus a short list of commonly used commands.

---

## 1. Install uv (Required for the Main Environment)

```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
```

Verify:

```bash
uv --version
```

---

## 2. Main Ψ₀ Python Environment (uv + `.venv-psi`)

Run the following from the **Psi0 repository root**. In this repository, that path is `Robot_Learning_Infra-WashU_Humanoid_Robot_Group/Psi0`:

```bash
cd /home/humanoid-pc/yu.zihao/Robot_Learning_Infra-WashU_Humanoid_Robot_Group/Psi0
```

### 2.1 Minimal G1 Fine-Tuning / Server Dependencies

This follows the upstream README "Installation" flow.

```bash
uv venv .venv-psi --python 3.10
source .venv-psi/bin/activate
GIT_LFS_SKIP_SMUDGE=1 uv sync \
  --group serve \
  --group viz \
  --group psi \
  --index-strategy unsafe-best-match \
  --active
uv pip install flash_attn==2.7.4.post1 --no-build-isolation
```

### 2.2 Full Dependency Groups for SIMPLE Evaluation, etc.

```bash
git submodule update --init --recursive
GIT_LFS_SKIP_SMUDGE=1 uv sync --all-groups --index-strategy unsafe-best-match --active
uv pip install flash_attn==2.7.4.post1 --no-build-isolation
UV_PROJECT_ENVIRONMENT=$(pwd)/.venv-psi ./scripts/install_curobo.sh
```

### 2.3 Quick Sanity Check

```bash
source .venv-psi/bin/activate
python -c "import psi; print(psi.__version__)"
python -c "from psi.data.lerobot.compat import LEROBOT_LAYOUT; print(LEROBOT_LAYOUT)"
```

---

## 3. Independent uv Environments for Subpackages (Optional)

Some baselines / subprojects have their own `pyproject.toml` files. With **`.venv-psi` already activated**, or inside a separate virtual environment, you can follow the upstream documentation and run `uv sync` inside those subdirectories, such as `src/h_rdt`, `src/egovla`, or `src/gr00t`.

The repository test script `Psi0/scripts/test_regression.py` may reference interpreters from multiple paths. For daily **core Ψ₀** work, prefer the repository-root **uv + `.venv-psi`** environment.

---

## 4. Real-Robot Deployment / Data Collection (conda, `Psi0/real/` Only)

Keep this **separate** from the training **uv** environment. For `Psi0/real/`, follow the official instructions and use **conda**:

```bash
cd /home/humanoid-pc/yu.zihao/Robot_Learning_Infra-WashU_Humanoid_Robot_Group/Psi0/real
conda env create -f psi_deploy_env.yaml
conda activate psi_deploy
# Then follow Psi0/real/README.md to install unitree_sdk2_python,
# run pip install -e ., etc.
```

The camera service on the robot may use another conda environment, such as `vision`. Again, use **`Psi0/real/README.md`** as the source of truth.

---

## 5. Summary: Should I Use uv or conda?

- **Psi0 model training, fine-tuning, server usage, and development aligned with the repository `pyproject.toml`**: use **uv**, with the virtual environment at **`.venv-psi`** (Python 3.10).
- **Real-robot teleop / RTC / deployment workflow (`real/`)**: use **conda**, with the environment named **`psi_deploy`** (created from `psi_deploy_env.yaml`).
- **Do not** mix the two workflows in the same environment unless you fully understand the dependency-conflict risk.

---

## 6. One-Sentence Rule

**Use uv (`.venv-psi`) for the main Psi0 codebase; use conda (`psi_deploy`) for real-robot deployment under `real/`.**
