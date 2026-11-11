#!/usr/bin/env python3
"""Step 5: IMU / Camera SOF qtimer alignment report (35s joint capture)."""
import re, sys
import numpy as np

IMU_CSV = "/tmp/imu_35s.csv"
CAM_LOG = "/tmp/cam_35s.log"

# --- parse IMU ---
recs = {"gyro": [], "accel": []}
with open(IMU_CSV) as f:
    next(f)
    for line in f:
        p = line.strip().split(",")
        if len(p) != 6:
            continue
        recs[p[0]].append((float(p[1]), float(p[2]), float(p[3]),
                           int(round(int(p[4])*1000/19.2)), int(p[5])))

for k in recs:
    a = np.array(recs[k], dtype=object)
    ts = np.array([r[3] for r in recs[k]], dtype=np.int64)
    arr = np.array([r[4] for r in recs[k]], dtype=np.int64)
    d = np.diff(ts)
    lat = (arr - ts) / 1e6
    print(f"[{k}] N={len(ts)} span={ts[-1]-ts[0]:.3e}ns "
          f"({(ts[-1]-ts[0])/1e9:.2f}s) mean_dt={d.mean()/1000:.0f}us "
          f"dt_p99={np.percentile(d,99)/1000:.0f}us "
          f"monotonic={bool((d>0).all())} "
          f"lat_med={np.median(lat):.2f}ms lat_p99={np.percentile(lat,99):.2f}ms")

gy_ts = np.array([r[3] for r in recs["gyro"]], dtype=np.int64)
gy = np.array([[r[0], r[1], r[2]] for r in recs["gyro"]])
ac_ts = np.array([r[3] for r in recs["accel"]], dtype=np.int64)
ac = np.array([[r[0], r[1], r[2]] for r in recs["accel"]])

# --- parse camera log ---
sof_re = re.compile(r"requestID: (\d+), QTimer: SOFTimestamp: (\d+), "
                    r"exposure start time: (\d+)")
mono_re = re.compile(r"frameMessage:requestID=(\d+), .*frameCount=(\d+), "
                     r"timestampType=1, Timestamp=(\d+)")
sof, exp = [], []
mono = {}
t0 = None
with open(CAM_LOG, errors="replace") as f:
    for line in f:
        if "QTimer: SOFTimestamp" in line:
            m = sof_re.search(line)
            if m:
                sof.append(int(m.group(2)))
                exp.append(int(m.group(3)))
        elif "timestampType=1" in line:
            m = mono_re.search(line)
            if m:
                mono[int(m.group(1))] = int(m.group(3))

sof = np.array(sof, dtype=np.int64)
exp = np.array(exp, dtype=np.int64)
print(f"\n[camera] SOF frames={len(sof)} span={(sof[-1]-sof[0])/1e9:.2f}s "
      f"fps={len(sof)/((sof[-1]-sof[0])/1e9):.2f}")
d = np.diff(sof)
print(f"[camera] frame dt: mean={d.mean()/1e6:.2f}ms p1={np.percentile(d,1)/1e6:.2f}ms "
      f"p99={np.percentile(d,99)/1e6:.2f}ms jitter_std={d.std()/1e6:.3f}ms")

# mono vs qtimer offset
offs = []
for rid, q in enumerate(sof):
    if rid in mono and mono[rid] != 0:
        offs.append(q - mono[rid])
if offs:
    offs = np.array(offs, dtype=np.int64)
    print(f"[offset] Qtimer-Mono over {len(offs)} frames: "
          f"med={np.median(offs)/1e9:.6f}s std={offs.std()/1e6:.3f}ms")

# --- coverage: IMU samples within +-16.7ms of each SOF ---
W = 16666667
cov = np.array([np.sum(np.abs(gy_ts - s) <= W) for s in sof])
print(f"\n[coverage] gyro samples per frame window +-16.7ms: "
      f"min={cov.min()} med={np.median(cov):.0f} max={cov.max()} "
      f"frames_with_zero={(cov==0).sum()}/{len(sof)}")

# --- shake correlation: gyro magnitude vs time, per-frame ---
gm = np.linalg.norm(gy, axis=1)
# average |gyro| in each frame's exposure window
frame_g = []
for s in sof:
    m = (gy_ts >= s - W) & (gy_ts <= s + W)
    frame_g.append(gm[m].mean() if m.any() else np.nan)
frame_g = np.array(frame_g)
valid = ~np.isnan(frame_g)
print(f"\n[shake] frames with gyro data: {valid.sum()}/{len(sof)}")
print(f"[shake] per-frame mean |gyro|: min={np.nanmin(frame_g):.4f} "
      f"max={np.nanmax(frame_g):.4f} rad/s")
# segment activity into quiet/shake using threshold
thr = np.nanmean(frame_g) + 2*np.nanstd(frame_g)
shake_mask = frame_g > thr
print(f"[shake] threshold={thr:.4f} rad/s, shake frames={shake_mask.sum()}")
# contiguous shake segments (in seconds from first SOF)
segs = []
i = 0
while i < len(shake_mask):
    if shake_mask[i]:
        j = i
        while j < len(shake_mask) and shake_mask[j]:
            j += 1
        if j - i >= 5:  # >=~0.17s
            segs.append(((sof[i]-sof[0])/1e9, (sof[j-1]-sof[0])/1e9))
        i = j
    else:
        i += 1
print(f"[shake] detected shake segments (rel to first SOF):")
for a, b in segs:
    print(f"    {a:6.2f}s - {b:6.2f}s  ({b-a:.2f}s)")
