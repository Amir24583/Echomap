# EchoMap

Echolocation in the browser. EchoMap turns a phone's speaker and microphone into a short-range sonar. It plays a chirp, listens for echoes, and maps the room as you turn. No app install, no extra hardware. Built for Safari on iPhone, and works in any modern browser with a mic.

Open it over HTTPS (GitHub Pages works). The microphone is blocked on plain `http://` pages, except on localhost.

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

## Development

```bash
node test/sim.test.js          # simulated echoes at 0.6 / 1.8 / 3.2 m with random latency; asserts < 3 cm error
python3 -m http.server 8765    # then open http://localhost:8765
```

`dsp.js` holds pure signal-processing functions (no DOM) and is shared by the page and the tests. `app.js` holds audio I/O, displays, the test mode and export.

Planned: LiDAR fusion on phones that have it, and more features.
