import csv
import glob
import os
import socket
import sys
import time
import cv2
import numpy as np


def find_camera_index():
    if len(sys.argv) > 1 and sys.argv[1].isdigit():
        idx = int(sys.argv[1])
        print(f"using manual camera index {idx}")
        return idx

    for idx in range(5):
        cap = cv2.VideoCapture(idx, cv2.CAP_AVFOUNDATION)
        if cap.isOpened():
            ret, _ = cap.read()
            cap.release()
            if ret:
                print(f"selected camera index {idx}")
                return idx

    print("falling back to camera index 0")
    return 0


def get_next_cv_filename():
    existing = glob.glob("cv_data_*.csv")
    indices = []
    for f in existing:
        try:
            num = int(f.split("_")[-1].split(".")[0])
            indices.append(num)
        except ValueError:
            pass
    next_idx = max(indices) + 1 if indices else 1
    return f"cv_data_{next_idx:03d}.csv"


def main():
    calib_file = "camera_calib.npz"
    if not os.path.exists(calib_file):
        print("error: calibration file 'camera_calib.npz' missing")
        sys.exit(1)

    cam_index = find_camera_index()
    log_filename = get_next_cv_filename()

    esp_ip = "192.168.4.1"
    udp_port = 4242
    marker_size = 0.10
    cog_offset = np.array([-0.004, -0.0381, 0.00], dtype=np.float32)

    alpha_pos = 0.7
    alpha_vel = 0.5

    calib = np.load(calib_file)
    camera_matrix = calib["camera_matrix"]
    dist_coeffs = calib["dist_coeffs"]

    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    detector = cv2.aruco.ArucoDetector(aruco_dict, cv2.aruco.DetectorParameters())

    half_s = marker_size / 2.0
    marker_3d = np.array([
        [-half_s, half_s, 0.0],
        [half_s, half_s, 0.0],
        [half_s, -half_s, 0.0],
        [-half_s, -half_s, 0.0]
    ], dtype=np.float32)

    cap = cv2.VideoCapture(cam_index, cv2.CAP_AVFOUNDATION)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

    if not cap.isOpened():
        print("error: could not open video stream")
        sys.exit(1)

    # UDP Sync Handshake
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(1.5)  # 1.5 second timeout to wait for ACK
    host_time_us = int(time.time() * 1e6)
    start_cmd = f"START:{host_time_us}"

    esp_synced = False
    try:
        sock.sendto(start_cmd.encode(), (esp_ip, udp_port))
        print(f"sent start command ({start_cmd}) to ESP32...")
        data, _ = sock.recvfrom(1024)
        response = data.decode().strip()
        print(f"ESP32 response: {response}")
        esp_synced = True
        print(">>> STATUS: ESP32 & MACBOOK SYNCHRONIZED SUCCESSFULLY <<<")
    except socket.timeout:
        print("WARNING: No response/ACK from ESP32. Check Wi-Fi connection to ESP32 AP.")
    except Exception as err:
        print(f"UDP sync error: {err}")

    sock.settimeout(None)  # Reset socket timeout

    print(f"saving CV position telemetry to: {log_filename}")

    f_log = open(log_filename, "w", newline="")
    writer = csv.writer(f_log)
    writer.writerow([
        "timestamp_unix_us", "tag_id",
        "x_raw", "y_raw", "z_raw",
        "x_filt", "y_filt", "z_filt",
        "roll_rad", "pitch_rad", "yaw_rad",
        "vx_filt", "vy_filt", "vz_filt"
    ])

    subpix_criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.01)
    pos_filtered = None
    vel_filtered = np.zeros(3, dtype=np.float32)
    prev_pos = None
    prev_ts = None

    start_time_sec = time.time()
    print("tracking started, press q to stop trial")

    try:
        while cap.isOpened():
            ret, frame = cap.read()
            if not ret:
                continue

            ts_us = int(time.time() * 1e6)
            elapsed_sec = time.time() - start_time_sec
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = detector.detectMarkers(gray)

            if ids is not None:
                ids_flat = ids.flatten()
                for i, tag_id in enumerate(ids_flat):
                    pts = corners[i][0].astype(np.float32)
                    pts = cv2.cornerSubPix(gray, pts, (5, 5), (-1, -1), subpix_criteria)

                    ok, rvec, tvec = cv2.solvePnP(
                        marker_3d, pts, camera_matrix, dist_coeffs, flags=cv2.SOLVEPNP_IPPE_SQUARE
                    )

                    if ok:
                        R, _ = cv2.Rodrigues(rvec)
                        t_cog = tvec.flatten() + (R @ cog_offset)

                        if pos_filtered is None:
                            pos_filtered = t_cog.copy()
                        else:
                            pos_filtered = alpha_pos * t_cog + (1.0 - alpha_pos) * pos_filtered

                        ts_sec = ts_us / 1e6
                        if prev_pos is not None and prev_ts is not None:
                            dt = ts_sec - prev_ts
                            if dt > 0.001:
                                vel_raw = (pos_filtered - prev_pos) / dt
                                vel_filtered = alpha_vel * vel_raw + (1.0 - alpha_vel) * vel_filtered

                        prev_pos = pos_filtered.copy()
                        prev_ts = ts_sec

                        roll = np.arctan2(R[2, 1], R[2, 2])
                        pitch = np.arctan2(-R[2, 0], np.sqrt(R[2, 1]**2 + R[2, 2]**2))
                        yaw = np.arctan2(R[1, 0], R[0, 0])

                        writer.writerow([
                            ts_us, tag_id,
                            f"{t_cog[0]:.5f}", f"{t_cog[1]:.5f}", f"{t_cog[2]:.5f}",
                            f"{pos_filtered[0]:.5f}", f"{pos_filtered[1]:.5f}", f"{pos_filtered[2]:.5f}",
                            f"{roll:.5f}", f"{pitch:.5f}", f"{yaw:.5f}",
                            f"{vel_filtered[0]:.5f}", f"{vel_filtered[1]:.5f}", f"{vel_filtered[2]:.5f}"
                        ])

                        cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec, tvec, half_s)

            # --- ON-SCREEN HUD OVERLAY ---
            overlay = frame.copy()
            cv2.rectangle(overlay, (10, 10), (550, 125), (0, 0, 0), -1)
            cv2.addWeighted(overlay, 0.6, frame, 0.4, 0, frame)

            sync_str = "ESP32: SYNCED" if esp_synced else "ESP32: NO ACK"
            sync_color = (0, 255, 0) if esp_synced else (0, 0, 255)

            cv2.putText(frame, f"Time: {elapsed_sec:.2f} s | {sync_str}", (20, 35),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, sync_color, 2)

            if pos_filtered is not None:
                cv2.putText(frame, f"Pos (m): X:{pos_filtered[0]:.3f} Y:{pos_filtered[1]:.3f} Z:{pos_filtered[2]:.3f}", (20, 65),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                cv2.putText(frame, f"Vel (m/s): X:{vel_filtered[0]:.3f} Y:{vel_filtered[1]:.3f} Z:{vel_filtered[2]:.3f}", (20, 95),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
            else:
                cv2.putText(frame, "Pos: SEARCHING FOR ARUCO MARKER...", (20, 65),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 2)

            cv2.imshow("cv tracking stream", cv2.resize(frame, (960, 540)))
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

    finally:
        try:
            sock.sendto(b"STOP", (esp_ip, udp_port))
            print("sent stop command to esp32")
        except Exception as err:
            print(f"udp stop error: {err}")

        f_log.close()
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
