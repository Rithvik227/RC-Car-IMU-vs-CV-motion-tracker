"""Merge onboard IMU telemetry with smartphone CV tracking and compute error metrics.

Corrected version (Oct 26, 2026). Changes from the original script:
  - IMU and CV files are paired by their shared laptop-clock start timestamp, not by file number
    (imu_run_001 pairs with cv_data_002 and imu_run_002 with cv_data_001).
  - Run 006 is excluded: imu_run_006 and cv_data_006 were recorded 48 min apart and no matching
    file for either exists.
  - Only ArUco Tag ID 0 is used; detections of other tag IDs are discarded and the EMA filter is
    re-applied offline to Tag 0 rows only.
  - CV and IMU are aligned on the synchronized Unix timestamps (ESP32 clock set from the host at START)
    instead of by matching motion-onset thresholds.
  - Integration uses the trapezoidal rule.
  - The IMU sign is taken from the acceleration at motion onset (no trial required inversion).
  - The Run 004 hard-coded override is removed; every CV value is the camera's own measurement.
  - The constant -0.004 m term is dropped because it cancels when the start position is zeroed.
"""
import sys
import os
import numpy as np
import pandas as pd
from scipy.signal import butter, filtfilt, correlate
from scipy.integrate import cumulative_trapezoid

# trial -> (imu file id, cv file id); paired by matching start timestamps
PAIRS = {
    "001": ("001", "002"), "002": ("002", "001"), "003": ("003", "003"), "004": ("004", "004"),
    "005": ("005", "005"), "007": ("007", "007"), "008": ("008", "008"),
    "009": ("009", "009"), "010": ("010", "010"), "011": ("011", "011"), "012": ("012", "012"),
}
EXCLUDED = {"006": "imu_run_006 and cv_data_006 start 2901 s apart; no matching files"}

CV_SCALE = 1.20          # 120 mm printed marker / 100 mm marker size assumed by cv_tracker.py
ALPHA_POS = 0.7          # EMA weight used by cv_tracker.py
MEDIAN_WIN = 15          # centered rolling median (frames)
LPF_CUTOFF_HZ = 10.0     # 2nd-order Butterworth, zero phase
STATIONARY_SEC = 1.5     # bias window and velocity clamp
ONSET_ACC = 0.20         # m/s^2, onset threshold (5 consecutive samples)
MAX_LAG_SEC = 0.5        # cross-correlation search window


def process_single_pair(trial_id, imu_path, cv_path, out_dir):
    imu_df = pd.read_csv(imu_path)
    cv_df = pd.read_csv(cv_path)

    start_gap = (cv_df["timestamp_unix_us"].iloc[0] - imu_df["timestamp_unix_us"].iloc[0]) / 1e6
    if abs(start_gap) > 1.0:
        raise ValueError(f"trial {trial_id}: IMU and CV files start {start_gap:.1f} s apart")

    # common time base: laptop clock, zero at first IMU sample
    t0 = imu_df["timestamp_unix_us"].iloc[0]
    t = (imu_df["timestamp_unix_us"].values - t0) / 1e6
    imu_df["time_sec"] = t
    dt = np.diff(t)
    fs = 1.0 / np.mean(dt)

    # forward axis = logged accel_y (sensor x-axis after firmware remap)
    b, a = butter(N=2, Wn=LPF_CUTOFF_HZ / (fs / 2.0), btype="low")
    accel_lpf = filtfilt(b, a, imu_df["accel_y"].values)

    stationary = t <= STATIONARY_SEC
    bias = np.mean(accel_lpf[stationary])
    accel_clean = accel_lpf - bias

    # sign from acceleration at sustained motion onset
    above = np.abs(accel_clean) > ONSET_ACC
    run5 = np.convolve(above, np.ones(5, dtype=int), "valid") == 5
    onset_idx = int(np.argmax(run5))
    sign = 1.0 if accel_clean[onset_idx] >= 0 else -1.0
    accel_clean *= sign

    # trapezoidal integration; velocity clamped to zero during the stationary window
    accel_int = np.where(stationary, 0.0, accel_clean)
    vel = cumulative_trapezoid(accel_int, t, initial=0.0)
    vel[stationary] = 0.0
    pos = cumulative_trapezoid(vel, t, initial=0.0)

    imu_df["imu_accel_x_filt"] = accel_clean
    imu_df["imu_vel_x"] = vel
    imu_df["imu_pos_x"] = pos

    # CV: Tag ID 0 only, EMA re-applied, centered rolling median, scale correction
    n_other = int((cv_df["tag_id"] != 0).sum())
    cv0 = cv_df[cv_df["tag_id"] == 0].reset_index(drop=True)
    tc = (cv0["timestamp_unix_us"].values - t0) / 1e6
    x_raw = cv0["x_raw"].values
    ema = np.empty_like(x_raw)
    ema[0] = x_raw[0]
    for i in range(1, len(x_raw)):
        ema[i] = ALPHA_POS * x_raw[i] + (1.0 - ALPHA_POS) * ema[i - 1]
    x_med = pd.Series(ema).rolling(MEDIAN_WIN, center=True, min_periods=1).median().values
    x_scaled = CV_SCALE * x_med

    cv_interp = np.interp(t, tc, x_scaled, left=x_scaled[0], right=x_scaled[-1])
    cv_pos = cv_interp - np.mean(cv_interp[stationary])
    if cv_pos[-1] < 0:          # camera x-axis points opposite to travel in some setups
        cv_pos = -cv_pos

    # residual offset: cross-correlation of IMU and CV velocities within +/-0.5 s
    cv_vel = np.gradient(cv_pos, t)
    s1 = vel - vel.mean()
    s2 = cv_vel - cv_vel.mean()
    corr = correlate(s1, s2, mode="full")
    lags = np.arange(-len(s1) + 1, len(s1))
    win = np.abs(lags) <= int(MAX_LAG_SEC * fs)
    best_lag = int(lags[win][np.argmax(corr[win])])
    lag_sec = best_lag / fs   # negative = CV trace lags IMU

    # remove the residual offset: shift the CV time base by the measured lag and re-zero
    cv_interp = np.interp(t, tc + lag_sec, x_scaled, left=x_scaled[0], right=x_scaled[-1])
    cv_pos = cv_interp - np.mean(cv_interp[stationary])
    if cv_pos[-1] < 0:
        cv_pos = -cv_pos
    cv_vel2 = np.gradient(cv_pos, t)
    c2 = correlate(s1, cv_vel2 - cv_vel2.mean(), mode="full")
    residual_lag = int(lags[win][np.argmax(c2[win])]) / fs

    # motion onset gap on the shared clock (CV first moves 1 cm vs IMU onset)
    cv_dev = np.abs(x_scaled - np.median(x_scaled[:10]))
    cv_run = np.convolve(cv_dev > 0.01, np.ones(5, dtype=int), "valid") == 5
    cv_onset_t = tc[int(np.argmax(cv_run))]
    onset_gap = cv_onset_t - t[onset_idx]

    imu_df["cv_pos_x_aligned"] = cv_pos
    imu_df["position_error_m"] = np.abs(pos - cv_pos)

    out_path = os.path.join(out_dir, f"research_telemetry_merged_{trial_id}.csv")
    imu_df.to_csv(out_path, index=False)

    # stop time from CV: last time CV differs from its final value by > 1 cm
    moving = np.where(np.abs(cv_pos - cv_pos[-1]) > 0.01)[0]
    t_stop = t[moving[-1]] if len(moving) else np.nan
    rest = t > t_stop + 0.3
    gyro = np.sqrt(imu_df["gyro_x"] ** 2 + imu_df["gyro_y"] ** 2 + imu_df["gyro_z"] ** 2)

    return {
        "trial": trial_id, "imu_file": os.path.basename(imu_path), "cv_file": os.path.basename(cv_path),
        "start_gap_s": start_gap, "fs_hz": fs, "median_dt_ms": np.median(dt) * 1e3, "max_dt_ms": dt.max() * 1e3,
        "dup_rows_pct": 100 * (imu_df[["accel_x", "accel_y", "accel_z", "gyro_x", "gyro_y", "gyro_z"]]
                               .diff().abs().sum(axis=1) == 0).mean(),
        "cv_rows_per_s": len(cv0) / (tc[-1] - tc[0]), "cv_other_tag_rows": n_other,
        "bias_ms2": bias, "sign": sign, "imu_onset_s": t[onset_idx], "cv_onset_gap_s": onset_gap,
        "vel_lag_s": lag_sec, "residual_lag_after_s": residual_lag, "t_stop_s": t_stop, "post_stop_s": t[-1] - t_stop,
        "end_vel_ms": vel[-1], "rest_cv_sd_mm": 1e3 * np.std(cv_pos[rest]) if rest.sum() > 10 else np.nan,
        "gyro_peak_rads": gyro.max(), "accel_clipped": bool((imu_df["accel_y"].abs() >= 39.19).any()),
        "cv_final_m": cv_pos[-1], "imu_final_m": pos[-1],
    }


def main():
    data_dir = sys.argv[1] if len(sys.argv) > 1 else "."
    out_dir = sys.argv[2] if len(sys.argv) > 2 else "."
    os.makedirs(out_dir, exist_ok=True)
    rows = []
    for trial, (ik, ck) in PAIRS.items():
        rows.append(process_single_pair(trial, os.path.join(data_dir, f"imu_run_{ik}.csv"),
                                        os.path.join(data_dir, f"cv_data_{ck}.csv"), out_dir))
    diag = pd.DataFrame(rows)
    diag.to_csv(os.path.join(out_dir, "processing_diagnostics.csv"), index=False)
    for k, why in EXCLUDED.items():
        print(f"excluded trial {k}: {why}")
    pd.set_option("display.width", 250)
    print(diag.round(4).to_string(index=False))


if __name__ == "__main__":
    main()
