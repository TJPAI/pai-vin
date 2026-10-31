# PAI-VIN V0.2 — iPhone offline feasibility prototype

**Status:** source pushed to main. GitHub Pages deployment has not yet been independently verified.

## What can be tested on a real iPhone
1. Open the GitHub Pages URL over HTTPS in Safari.
2. Tap **开启 IMU** and grant permission.
3. Tap **打开摄像头**, grant permission, and save an image anchor while viewing a textured scene.
4. Move the phone, return to the same viewpoint, and tap **尝试匹配**.
5. Test **关闭摄像头 / 口袋**. Observe whether motion samples continue while Safari stays in the foreground.
6. Use **自动举起检测** only as an experimental device-orientation trigger; it may fail or false-trigger.
7. Export JSON logs and share results for algorithm tuning.
8. Test airplane mode only after the page and scripts have successfully loaded once.

## V0.2 implementation
- 2D binary local features and translation-consistency match gate (vision.js)
- Motion sensor permission and basic non-calibrated step detector
- Experimental orientation-based raise-to-scan detection, with separate permission request
- Offline script precache, local anchor persistence, JSON measurement export
- JavaScript syntax checks before Pages deployment

## Important limitations
- **Not 3D SLAM, 2D–3D PnP, calibrated localization, or a navigation map.**
- Matched image anchors use unvalidated estimated trajectory coordinates.
- No automatic building mapping, floor recognition, full PDR calibration, or route planning.
- Images are processed in the browser and not uploaded by this application.
- Gyro integration uses a simple approximation; step length is fixed at 0.68m.
- Magnetometer and barometer readings are not available through the current PWA sensor adapter.
- Safari may suspend motion events if backgrounded or screen locked.

## Planned demonstration
Target date: November 7, 2026.
The customer-facing demo must clearly distinguish measured, inferred, and simulated content.
