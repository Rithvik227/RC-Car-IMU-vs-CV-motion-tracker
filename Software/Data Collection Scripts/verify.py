import glob
import sys
import pandas as pd

def verify():
    # 1. Parse command-line args or auto-detect latest files
    if len(sys.argv) >= 3:
        latest_imu = sys.argv[1]
        latest_cv = sys.argv[2]
    else:
        imu_files = sorted(glob.glob("imurun.csv"))
        cv_files = sorted(glob.glob("cv_groundtruth.csv"))

        if not imu_files or not cv_files:
            print("error: missing telemetry files in current directory")
            return

        latest_imu = imu_files[-1]
        latest_cv = cv_files[-1]

    print(f"evaluating trial files: {latest_imu} and {latest_cv}")

    try:
        imu_df = pd.read_csv(latest_imu)
        cv_df = pd.read_csv(latest_cv)
    except Exception as err:
        print(f"error loading csv files: {err}")
        return

    print(f"imu samples recorded: {len(imu_df)}")
    print(f"cv samples recorded: {len(cv_df)}")

    if len(imu_df) == 0 or len(cv_df) == 0:
        print("error: one or both telemetry files are empty")
        return

    # 2. Extract timestamps in microseconds
    imu_t_start = imu_df["timestamp_unix_us"].iloc[0]
    imu_t_end = imu_df["timestamp_unix_us"].iloc[-1]
    cv_t_start = cv_df["timestamp_unix_us"].iloc[0]
    cv_t_end = cv_df["timestamp_unix_us"].iloc[-1]

    print(f"imu start timestamp: {imu_t_start} us")
    print(f"cv start timestamp:  {cv_t_start} us")

    # 3. Measure initial sync delay and total overlapping capture window
    diff_start_ms = (imu_t_start - cv_t_start) / 1000.0
    print(f"initial timestamp difference: {diff_start_ms:.2f} ms")

    overlap_start = max(imu_t_start, cv_t_start)
    overlap_end = min(imu_t_end, cv_t_end)
    overlap_sec = (overlap_end - overlap_start) / 1e6

    if overlap_sec > 0:
        print(f"sync check passed: datasets overlap for {overlap_sec:.2f} seconds")
    else:
        print("sync check failed: no overlapping timestamp window found")

if __name__ == "__main__":
    verify()
