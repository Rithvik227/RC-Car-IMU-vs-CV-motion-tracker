import sys
import os
import numpy as np
import pandas as pd

def verify_sync(imu_path, cv_path):
    if not os.path.exists(imu_path):
        print(f"error: file '{imu_path}' not found")
        return
    if not os.path.exists(cv_path):
        print(f"error: file '{cv_path}' not found")
        return

    imu_df = pd.read_csv(imu_path)
    cv_df = pd.read_csv(cv_path)

    imu_ts_start = imu_df["timestamp_unix_us"].iloc[0]
    imu_ts_end = imu_df["timestamp_unix_us"].iloc[-1]
    cv_ts_start = cv_df["timestamp_unix_us"].iloc[0]
    cv_ts_end = cv_df["timestamp_unix_us"].iloc[-1]

    imu_dur = (imu_ts_end - imu_ts_start) / 1e6
    cv_dur = (cv_ts_end - cv_ts_start) / 1e6

    ts_diff_sec = (cv_ts_start - imu_ts_start) / 1e6

    imu_t = (imu_df["timestamp_unix_us"] - imu_ts_start) / 1e6
    cv_t = (cv_df["timestamp_unix_us"] - cv_ts_start) / 1e6

    accel_col = "accel_y" if "accel_y" in imu_df.columns else "accel_x"
    accel_raw = imu_df[accel_col].values
    bias = np.mean(accel_raw[imu_t <= 1.5]) if np.any(imu_t <= 1.5) else np.mean(accel_raw[:50])
    accel_dyn = np.abs(accel_raw - bias)
    imu_motion_mask = accel_dyn > 0.15

    if np.any(imu_motion_mask):
        imu_motion_start = imu_t[imu_motion_mask].iloc[0]
        imu_motion_end = imu_t[imu_motion_mask].iloc[-1]
    else:
        imu_motion_start, imu_motion_end = None, None

    cv_x = cv_df["x_filt"].values
    cv_dx = np.abs(cv_x - cv_x[0])
    cv_motion_mask = cv_dx > 0.05

    if np.any(cv_motion_mask):
        cv_motion_start = cv_t[cv_motion_mask].iloc[0]
        cv_motion_end = cv_t[cv_motion_mask].iloc[-1]
    else:
        cv_motion_start, cv_motion_end = None, None

    print(f"\n--- Synchronization Diagnostic for {imu_path} & {cv_path} ---")
    print(f"imu duration:                {imu_dur:.2f} s ({len(imu_df)} samples)")
    print(f"cv duration:                 {cv_dur:.2f} s ({len(cv_df)} samples)")
    print(f"raw timestamp difference:    {ts_diff_sec:.2f} s")

    if imu_motion_start is not None:
        print(f"imu motion phase (rel t=0):  {imu_motion_start:.2f} s to {imu_motion_end:.2f} s")
    else:
        print("imu motion phase:            no motion detected")

    if cv_motion_start is not None:
        print(f"cv motion phase (rel t=0):   {cv_motion_start:.2f} s to {cv_motion_end:.2f} s")
    else:
        print("cv motion phase:             no motion detected")

    if imu_motion_start is not None and cv_motion_start is not None:
        if cv_motion_start > imu_dur:
            print("verdict: MISMATCH - cv motion occurs after imu logging stopped")
        else:
            print("verdict: SYNC OK - both files capture overlapping motion")
    elif cv_motion_start is None:
        print("verdict: NO CV MOTION - vehicle did not move during cv recording window")

if __name__ == "__main__":
    if len(sys.argv) >= 3:
        imu_file = sys.argv[1]
        cv_file = sys.argv[2]
    else:
        imu_file = input("enter imu file path (e.g. imu_run_001.csv): ").strip()
        cv_file = input("enter cv file path (e.g. cv_ground_truth_001.csv): ").strip()

    verify_sync(imu_file, cv_file)
