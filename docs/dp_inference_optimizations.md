# Diffusion Policy Inference Optimizations

This note records runtime-only mitigations for cases where the policy appears
to get stuck during real-robot inference. These do not require retraining.

## Server-side sampler controls

### New optimization logic: sampler override

The DP server now accepts:

```bash
bash run_dp.sh server \
  --checkpoint /path/to/checkpoint.ckpt \
  --sampler ddim \
  --num-inference-steps 20 \
  --device cuda:0
```

- `--sampler ddpm` preserves the checkpoint's original DDPM scheduler.
- `--sampler ddim` swaps the runtime scheduler to DDIM from the checkpoint
  scheduler config.
- `--num-inference-steps N` overrides the policy sampling steps at inference
  time.

## Client-side stuck watchdog

### New optimization logic: repeated/low-motion chunk detection

The client now tracks quantized policy chunks and planned chunk motion. It can
detect two common symptoms:

- repeated chunks: the policy keeps producing the same action prefix;
- low-motion chunks: the selected chunk prefix has too little planned motion.

Default behavior is warning/logging only:

```bash
bash run_dp.sh client --ui --execute ...
```

To make the watchdog reset model history and skip the bad chunk:

```bash
bash run_dp.sh client \
  --ui \
  --execute \
  --auto-reset-on-stuck
```

Useful tuning flags:

```bash
--stuck-repeat-limit 3
--stuck-low-motion-limit 3
--stuck-eef-motion-eps 0.005
--stuck-joint-motion-eps 0.01
--stuck-signature-decimals 3
--reset-history-every 20
```

Use `--no-stuck-watchdog` to disable the logic.

