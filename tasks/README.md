# Task Presets and Splitter Rules

`task_list.json` is the source of truth for Gradio Step 1 task presets.
Add or edit tasks there instead of changing `gradio_panel.py`.

`splitter_list.json` is the source of truth for Gradio Episode Splitter modes.
Use it to define high-level splitter modes such as `forward` and `backward`
segments for continuous data collection.
Each enabled splitter should also define a `raw_task.name`, so the original
long recording is saved in a separate raw directory instead of mixing with the
split output task directories.

Each task entry contains:

- `label`: visible name in the Gradio dropdown.
- `name`: saved task directory name under `xr_recordings/`.
- `goal`, `desc`, `steps`: text metadata written into each episode.
- `splitter`: optional declarative rule for splitting one long recording into shorter task episodes.

Current splitter idea for continuous bottle collection:

- `forward`: `place_bottle_into_paper_box`; start a segment when the robot is near the initial position and the selected hand goes from open to closed.
- `backward`: `take_bottle_out_of_paper_box`; take the bottle out from the paper box and return it near the start region.
- During new recordings, VR Left Y toggles the saved splitter marker between
  `forward` and `backward`. The splitter uses these markers first. Position
  and frame-gap thresholds are only the fallback for old data without markers.
  The marker resets to `forward` when a new episode starts and after an
  episode is saved.

The current splitter implementation is `utils/split_xr_episode.py`.
It uses the first open-to-close grasp event as the initial bottle region by
default, then compares later active-arm joint poses against that region.

Dry run:

```bash
python utils/split_xr_episode.py xr_recordings/<task>/episode_0001
```

Write split episodes:

```bash
python utils/split_xr_episode.py xr_recordings/<task>/episode_0001 \
  --splitter-id bottle_in_out --write
```

Optionally write split episodes and delete the original long episode after success:

```bash
python utils/split_xr_episode.py xr_recordings/<task>/episode_0001 \
  --splitter-id bottle_in_out --write --delete-source
```

By default, split episodes are written back under
`xr_recordings/<task_name>/raw_episode_xxxx/episode_yyyy`, where
`raw_episode_xxxx` records the source raw episode id.

In Gradio, choose `Bottle in/out continuous splitter` in Step 1 before
recording. The UI will switch the Task Name to `bottle_in_out_raw`. After the
long raw episode is saved, the UI will split it into the forward/backward task
directories and keep the raw episode as the source record.
