# COLMAP full end-to-end synthetic integration: verified success (2026-11-01)

**Actual GitHub Actions execution**, not a dry-run:
https://github.com/TJPAI/pai-vin/actions/runs/41289312431
Commit: `41b159cad683af47ee363aa0d3c83c5110951d65`.
Generated artifact: `colmap-synthetic-integration` (GitHub artifact ID `13076358559`, ~2.1 MB). The artifact contains `report.json`, `mapping_cloud.ply`, COLMAP TXT reconstruction and logs.

## Observed on GitHub-hosted Ubuntu 24.04, COLMAP 3.9.1

| Stage | Real result |
|---|---|
| Procedurally generated mapping MP4 | 30 frames at 5 FPS |
| Procedurally generated independent query MP4 | 12 frames at 5 FPS |
| Executed COLMAP feature extraction, matching and incremental mapping | Success |
| Mapping registered camera frames | **30 / 30 (100%)** |
| Separate mapping models | **1** |
| Generated nonempty sparse PLY model | **Yes**, CI assertion passed |
| Executed query feature extraction, exhaustive matching and image registration | Success |
| Held-out query frames registered to frozen mapping model | **12 / 12 (100%)** |
| Full GitHub Actions integration | **success** |
| Metric scale | **not established** |
| Surveyed P95 position error | **not measured** |

This verifies the **software orchestration and COLMAP output/report plumbing**. These are synthetic generated views, so registration rates must **never be described as real iPhone mapping/localization performance**.

## Remaining real-video blocker

The two private 42 s / 33 s user MP4s remain in the ChatGPT working environment, which lacks COLMAP and network package installation. The GitHub runner has COLMAP but does not automatically have these files. Never commit the original private indoor footage to a public GitHub repository. A consented controlled-access data path or a working local COLMAP runtime is required to obtain **real** reconstruction. Final 0.5 m P95 acceptance additionally requires calibration, physical scale and ground truth.
