"""Build Tables 1-2, Figures 2-4 and summary statistics from the corrected merged files.

Table and figure numbers match the manuscript. Run 006 is excluded (see process_and_merge.py).
"""
import sys
import os
import glob
import re
import json
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy import stats

TARGET = {"001": 0.75, "002": 0.75, "003": 0.75, "004": 0.75,
          "005": 1.00, "007": 1.00, "008": 1.00,
          "009": 1.50, "010": 1.50, "011": 1.50, "012": 1.50}


def main():
    d = sys.argv[1] if len(sys.argv) > 1 else "."
    recs = []
    for fp in sorted(glob.glob(os.path.join(d, "research_telemetry_merged_*.csv"))):
        k = re.search(r"(\d{3})", os.path.basename(fp)).group(1)
        df = pd.read_csv(fp)
        err = df["position_error_m"]
        cv_f, imu_f = df["cv_pos_x_aligned"].iloc[-1], df["imu_pos_x"].iloc[-1]
        recs.append({"Trial ID": f"Run {k}", "Target (m)": TARGET[k],
                     "CV (m)": cv_f, "IMU (m)": imu_f,
                     "CV Dev. (%)": abs(cv_f - TARGET[k]) / TARGET[k] * 100,
                     "Final Error (m)": err.iloc[-1], "Error (%)": err.iloc[-1] / cv_f * 100,
                     "MAE (m)": err.mean(), "RMSE (m)": np.sqrt((err ** 2).mean()),
                     "Peak Error (m)": err.max(), "df": df})
    T = pd.DataFrame(recs)

    # Table 2: individual trials (manuscript Table 2)
    cols = ["Trial ID", "Target (m)", "CV (m)", "IMU (m)", "CV Dev. (%)", "Final Error (m)",
            "Error (%)", "MAE (m)", "RMSE (m)", "Peak Error (m)"]
    t2 = T[cols].copy()
    for c in ["CV (m)", "IMU (m)", "Final Error (m)", "MAE (m)", "RMSE (m)", "Peak Error (m)"]:
        t2[c] = t2[c].round(3)
    for c in ["CV Dev. (%)", "Error (%)"]:
        t2[c] = t2[c].round(2)
    t2.to_csv(os.path.join(d, "table2_individual_trials.csv"), index=False)

    # Table 1: block summaries (manuscript Table 1)
    s = []
    for dist, g in T.groupby("Target (m)"):
        s.append({"Target (m)": dist, "Count": len(g),
                  "CV Travel (m)": f"{g['CV (m)'].mean():.3f} ± {g['CV (m)'].std():.3f}",
                  "IMU Travel (m)": f"{g['IMU (m)'].mean():.3f} ± {g['IMU (m)'].std():.3f}",
                  "CV Dev. (%)": f"{g['CV Dev. (%)'].mean():.2f} ± {g['CV Dev. (%)'].std():.2f}",
                  "Final Error (m)": f"{g['Final Error (m)'].mean():.3f} ± {g['Final Error (m)'].std():.3f}",
                  "Mean Error (%)": f"{g['Error (%)'].mean():.2f} ± {g['Error (%)'].std():.2f}",
                  "MAE (m)": f"{g['MAE (m)'].mean():.3f}", "RMSE (m)": f"{g['RMSE (m)'].mean():.3f}"})
    pd.DataFrame(s).to_csv(os.path.join(d, "table1_distance_summaries.csv"), index=False)

    # statistics
    cvdev, imuerr = T["CV Dev. (%)"], T["Error (%)"]
    imu_vs_target = (T["IMU (m)"] - T["Target (m)"]).abs() / T["Target (m)"] * 100
    n_better = int((imuerr > cvdev).sum())
    sign_p = stats.binomtest(n_better, len(T), 0.5).pvalue
    sign_p_target = stats.binomtest(int((imu_vs_target > cvdev).sum()), len(T), 0.5).pvalue
    wil = stats.wilcoxon(imuerr, cvdev)
    reg = stats.linregress(T["Target (m)"], T["Final Error (m)"])
    summary = {
        "N": len(T),
        "cv_dev_pct_mean": cvdev.mean(), "cv_dev_pct_sd": cvdev.std(), "cv_dev_pct_median": cvdev.median(),
        "cv_dev_pct_by_block": T.groupby("Target (m)")["CV Dev. (%)"].mean().round(2).to_dict(),
        "cv_dev_pct_mean_excl_004": cvdev[T["Trial ID"] != "Run 004"].mean(),
        "cv_travel_sd_by_block_cm": (T.groupby("Target (m)")["CV (m)"].std() * 100).round(2).to_dict(),
        "imu_err_pct_mean": imuerr.mean(), "imu_err_pct_sd": imuerr.std(), "imu_err_pct_median": imuerr.median(),
        "imu_err_pct_by_block": T.groupby("Target (m)")["Error (%)"].mean().round(2).to_dict(),
        "imu_err_pct_min": imuerr.min(), "imu_err_pct_max": imuerr.max(),
        "n_imu_err_over_50pct": int((imuerr > 50).sum()),
        "imu_vs_target_pct_mean": imu_vs_target.mean(), "imu_vs_target_pct_sd": imu_vs_target.std(),
        "imu_worse_than_cv_count": n_better, "sign_test_p": sign_p, "sign_test_vs_target_p": sign_p_target,
        "wilcoxon_p": wil.pvalue,
        "regression_slope": reg.slope, "regression_intercept": reg.intercept,
        "regression_r": reg.rvalue, "regression_p": reg.pvalue,
        "final_err_min_m": T["Final Error (m)"].min(), "final_err_max_m": T["Final Error (m)"].max(),
        "peak_err_max_m": T["Peak Error (m)"].max(),
        "mae_by_block": T.groupby("Target (m)")["MAE (m)"].mean().round(3).to_dict(),
        "rmse_by_block": T.groupby("Target (m)")["RMSE (m)"].mean().round(3).to_dict(),
    }
    with open(os.path.join(d, "summary_statistics.json"), "w") as f:
        json.dump(summary, f, indent=2, default=float)
    print(json.dumps(summary, indent=2, default=float))
    print(t2.to_string(index=False))
    print(pd.DataFrame(s).to_string(index=False))

    # Figure 2: representative overlays (first trial of each block)
    fig, axes = plt.subplots(1, 3, figsize=(15, 4.5))
    for i, (dist, run) in enumerate([(0.75, "Run 001"), (1.00, "Run 005"), (1.50, "Run 009")]):
        df = T.loc[T["Trial ID"] == run, "df"].iloc[0]
        ax = axes[i]
        ax.plot(df["time_sec"], df["cv_pos_x_aligned"], label="CV Position", color="blue", linewidth=2)
        ax.plot(df["time_sec"], df["imu_pos_x"], label="IMU Dead-Reckoning", color="red", linestyle="--", linewidth=2)
        ax.set_title(f"Target Distance: {dist:.2f} m ({run})", fontsize=11)
        ax.set_xlabel("Time (s)", fontsize=10)
        ax.set_ylabel("Position (m)", fontsize=10)
        ax.grid(True, linestyle=":", alpha=0.6)
        ax.legend(fontsize=9)
        ax.text(-0.12, 1.04, "ABC"[i] + ")", transform=ax.transAxes, fontsize=13, fontweight="bold")
    plt.tight_layout()
    plt.savefig(os.path.join(d, "figure2_trajectory_overlays.png"), dpi=300)
    plt.close()

    # Figure 3: kinematic profile, Run 005
    df = T.loc[T["Trial ID"] == "Run 005", "df"].iloc[0]
    fig, axes = plt.subplots(3, 1, figsize=(9, 7), sharex=True)
    axes[0].plot(df["time_sec"], df["imu_accel_x_filt"], color="purple", label="Filtered Acceleration")
    axes[0].set_ylabel("Accel (m/s²)", fontsize=10)
    axes[1].plot(df["time_sec"], df["imu_vel_x"], color="green", label="IMU Integrated Velocity")
    axes[1].set_ylabel("Velocity (m/s)", fontsize=10)
    axes[2].plot(df["time_sec"], df["imu_pos_x"], color="red", linestyle="--", label="IMU Position")
    axes[2].plot(df["time_sec"], df["cv_pos_x_aligned"], color="blue", label="CV Position")
    axes[2].set_ylabel("Position (m)", fontsize=10)
    axes[2].set_xlabel("Time (s)", fontsize=10)
    for j, ax in enumerate(axes):
        ax.grid(True, linestyle=":", alpha=0.6)
        ax.legend(fontsize=9)
        ax.text(-0.09, 1.0, "ABC"[j] + ")", transform=ax.transAxes, fontsize=12, fontweight="bold", va="bottom")
    plt.tight_layout()
    plt.savefig(os.path.join(d, "figure3_kinematic_profiles.png"), dpi=300)
    plt.close()

    # Figure 4: final error vs distance
    fig, ax = plt.subplots(figsize=(7, 5))
    ax.scatter(T["Target (m)"], T["Final Error (m)"], color="purple", s=60, alpha=0.8, zorder=3, label="Trial Drift Errors")
    g = T.groupby("Target (m)")["Final Error (m)"]
    ax.errorbar(g.mean().index, g.mean().values, yerr=g.std().values, fmt="o", color="darkred",
                capsize=5, capthick=2, linewidth=2, label="Mean ± Std Dev", zorder=4)
    fx = np.linspace(0.6, 1.6, 100)
    ax.plot(fx, reg.slope * fx + reg.intercept, color="black", linestyle="--",
            label=f"Fit: y = {reg.slope:.3f}x + {reg.intercept:.3f}")
    ax.set_xlabel("Target Motion Distance (m)", fontsize=11)
    ax.set_ylabel("Final Drift Error (m)", fontsize=11)
    ax.grid(True, linestyle=":", alpha=0.6)
    ax.legend(fontsize=10)
    plt.tight_layout()
    plt.savefig(os.path.join(d, "figure4_distance_error_scaling.png"), dpi=300)
    plt.close()


if __name__ == "__main__":
    main()
