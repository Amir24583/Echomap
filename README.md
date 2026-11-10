# EchoMap

Echolocation in the browser. EchoMap turns a phone's speaker and microphone into a short-range sonar. It plays a chirp, listens for echoes, and maps the room as you turn. No app install, no extra hardware. Built for Safari on iPhone, and works in any modern browser with a mic.

Open it over HTTPS (GitHub Pages works). The microphone is blocked on plain `http://` pages, except on localhost.

## Two modes

- **Sonar** (`index.html`): measures real distances with sound. Described below.
- **3D scan** (`scan.html`): builds a 3D point cloud of the room from the camera. See "3D scan" further down.
- **Livox LiDAR** (`livox/`, `viewer.html`): the accurate route. Python code that records a Livox Mid-360 and turns the data into a 3D map, with a simulator to test it before the hardware arrives. See "Livox LiDAR" further down.

## How it works

1. **Chirp.** A 4 ms linear sweep, either 18–22 kHz (quiet) or 4–10 kHz (audible, stronger on phone speakers), with tapered edges. One ping every 200 ms.
2. **Raw capture.** An AudioWorklet records the microphone with echo cancellation, noise suppression and auto-gain turned off, so the echoes survive. The last 30 s are kept for export.
3. **Matched filter.** FFT cross-correlation of the recording with the chirp, band-limited to the chirp band, analytic output (complex).
4. **Self-calibration.** The strongest arrival is the direct speaker-to-mic path, and it becomes "0 m" on every ping. That removes the unknown audio latency of the phone and browser. Its ideal response is then subtracted so its sidelobes don't hide nearby echoes.
5. **Averaging.** Several pings are averaged coherently (3 by default). Noise falls by about √N.
6. **Range.** r = c · Δt / 2, where c = 331.3 + 0.606·T m/s and T is the air temperature in Settings.
7. **Detection.** Cell-averaging CFAR. The guard band is sized to the matched-filter main lobe, so a peak's own shoulders don't hide it.
8. **Map.** Each detection is binned by relative heading (gyroscope yaw, 5° bins) and range (5 cm bins) on a polar grid.
9. **Motion.** "Capture background" stores the averaged complex profile of a still scene. The cyan trace is then |profile − background|, which shows only what moved.

## Checking it's right

The **Tape test** tab is the ground truth. Stand facing a flat wall at measured distances (0.5, 1, 2, 3 m) and press Measure. It reports the median, error and spread over 20 readings. "Use mean error as offset" corrects the fixed bias from the speaker and mic placement.

**Save raw data** exports a 16-bit WAV of the last 30 s plus a JSON file with every ping's sample index and heading, so the processing can be re-checked offline (for example in numpy/scipy).

## Limits

- A phone hears distance but not direction. Direction comes from you turning the phone, and the speaker's beam is wide, so walls come out as smeared arcs, not sharp lines.
- Range is about 3–5 m indoors. Soft furniture absorbs sound; hard walls reflect it well.
- Some phones roll off above 20 kHz. If the quiet band shows nothing, use the audible band.

## 3D scan

A phone with one mic can hear how far away things are, but not where they are, so sound alone can't make a 3D scan. The 3D mode uses the camera:

1. **Depth from one camera.** Each captured frame goes through Depth Anything V2 (small), running in the browser with transformers.js on WebGPU, or WebAssembly where WebGPU is missing. It returns relative inverse depth per pixel. The model (about 25–50 MB) is downloaded from Hugging Face on first use and cached.
2. **Pose from the gyroscope.** You stand on one spot and turn. Every pixel's viewing ray depends only on the phone's rotation, so each frame is a patch of one spherical depth panorama. A new frame is captured whenever you face more than 12° away from every earlier view and the phone is moving slowly enough.
3. **Alignment.** The network's scale and shift differ per frame. Each new frame is fitted (least squares, affine in inverse depth) to the panorama built so far over the directions they share, so the patches join without steps.
4. **Display.** Points are drawn with WebGL in camera colours or a height ramp. Drag to orbit, pinch to zoom. "Save .ply" exports the cloud for MeshLab, CloudCompare or Blender.

Limits: shapes come from an AI depth estimate, not a laser, and true size can't be measured with one camera. The scan is scaled so the middle of the first view sits at the "typical distance to walls" setting. Walking around is not tracked, only turning on the spot.

"Demo room" runs the whole pipeline on a synthetic room, with random scale and shift per frame, without a camera.

## Livox LiDAR

A phone can't measure distance with light, so a centimetre-accurate map needs a real LiDAR. `livox/` is the software for a Livox Mid-360, and it runs today against a simulated sensor.

```bash
pip install -r livox/requirements.txt
python -m livox.run_sim --out out/        # simulate a walk, build the map, score it against ground truth
python -m livox.test_livox                # packet format, ray casting, IMU, accuracy and UDP recorder checks
```

- `packets.py` reads and writes the sensor's UDP data packets (point cloud and IMU). The layout was checked against `livox_lidar_def.h` in Livox-SDK2.
- `sim.py` is a simulated Mid-360 carried through a virtual two-room building: 200,000 points/s, 360° × (−7°..52°) field of view, 2 cm range noise, 0.15° angular noise, walking bob and hand wobble, and a 200 Hz IMU. It emits the same packets as the real sensor.
- `record.py` listens on the sensor's ports (56301 points, 56401 IMU) and writes every datagram to a log, counting dropped packets.
- `mapper.py` turns a log into a map: 100 ms frames, LiDAR odometry with motion de-skew ([KISS-ICP](https://github.com/PRBonn/kiss-icp)), then a voxel-averaged point cloud saved as `.ply`.
- `run_sim.py` runs all of it and reports accuracy. The path is compared after the standard rigid alignment, and the map is compared with the building's true surfaces.
- `viewer.html` shows a `.ply` map in the browser (3D or top-down plan). It opens `livox/sample/sim-map.ply` by default.

Result for the full simulated walk (26 m, 40 s): median map error 1.7 cm, 99% of points within 5.9 cm, path error 3.2 cm RMS, 0.8 cm drift from start to end.

What the simulation does not cover: the sensor's start-up and network configuration (done once with Livox Viewer 2 or the SDK), its real scan pattern (approximated here), and real-world effects like glass, mirrors and moving people. The IMU is recorded but not yet used. Fusing it (as FAST-LIO does) is the next step if hand motion turns out to be the limit on the real unit.

With the real sensor: set the host IP on the sensor, run `python -m livox.record walk.lvx-log` on the Pi while you walk, then `python -m livox.mapper walk.lvx-log map.ply` on the laptop and open the result in `viewer.html`.

## Development

```bash
node test/sim.test.js          # simulated echoes at 0.6 / 1.8 / 3.2 m with random latency; asserts < 3 cm error
node test/geom.test.js         # 3D scan: orientation conventions, frame alignment on a synthetic room, PLY export
python3 -m http.server 8765    # then open http://localhost:8765
```

`dsp.js` holds pure signal-processing functions (no DOM) and is shared by the page and the tests. `app.js` holds audio I/O, displays, the test mode and export. `scan-geom.js` and `scan.js` are the same split for the 3D scan.

Planned: use the sonar distance to set the 3D scan's true scale, and LiDAR on phones that have it.
