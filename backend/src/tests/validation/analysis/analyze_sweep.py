import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import sys
import os

# -------------------------------------------------------------------
# Config
# -------------------------------------------------------------------

CSV_DIR = os.path.join(os.path.dirname(__file__), "csv_data")
DEFAULT_FILE = "vbar_sweep.csv"

# -------------------------------------------------------------------
# Load
# -------------------------------------------------------------------

def load_sweep(filepath):
    df = pd.read_csv(filepath)
    print(f"Loaded {len(df)} samples from {filepath}")
    print(f"Q_pos range: [{df['Q_pos'].min():.2f}, {df['Q_pos'].max():.2f}]")
    print(f"Q_vel range: [{df['Q_vel'].min():.2f}, {df['Q_vel'].max():.2f}]")
    print(f"Unique Q_pos: {df['Q_pos'].nunique()}, Unique Q_vel: {df['Q_vel'].nunique()}")
    return df

# -------------------------------------------------------------------
# Aggregate per grid cell
# -------------------------------------------------------------------

def aggregate_grid(df):
    grouped = df.groupby(["Q_pos", "Q_vel"]).agg(
        success_rate=("success", "mean"),
        mean_dv=("total_dv", "mean"),
        p95_dv=("total_dv", lambda x: np.percentile(x, 95)),
        mean_duration=("duration", "mean"),
        n_samples=("success", "count"),
    ).reset_index()

    return grouped

# -------------------------------------------------------------------
# Build 2D grid for heatmap
# -------------------------------------------------------------------

def build_heatmap_grid(grouped, value_col):
    Q_pos_vals = np.sort(grouped["Q_pos"].unique())
    Q_vel_vals = np.sort(grouped["Q_vel"].unique())

    grid = np.full((len(Q_pos_vals), len(Q_vel_vals)), np.nan)

    pos_idx = {v: i for i, v in enumerate(Q_pos_vals)}
    vel_idx = {v: i for i, v in enumerate(Q_vel_vals)}

    for _, row in grouped.iterrows():
        i = pos_idx[row["Q_pos"]]
        j = vel_idx[row["Q_vel"]]
        grid[i, j] = row[value_col]

    return Q_pos_vals, Q_vel_vals, grid

# -------------------------------------------------------------------
# Plots
# -------------------------------------------------------------------

def plot_success_rate_heatmap(grouped, title_suffix=""):
    Q_pos_vals, Q_vel_vals, grid = build_heatmap_grid(grouped, "success_rate")

    fig, ax = plt.subplots(figsize=(10, 8))
    im = ax.pcolormesh(
        Q_vel_vals, Q_pos_vals, grid,
        cmap="RdYlGn", vmin=0.0, vmax=1.0, shading="nearest"
    )

    ax.set_xscale("log")
    ax.set_yscale("log")
    ax.set_xlabel("Q_vel", fontsize=12)
    ax.set_ylabel("Q_pos", fontsize=12)
    ax.set_title(f"Success Rate{title_suffix}", fontsize=14)

    cbar = fig.colorbar(im, ax=ax)
    cbar.set_label("Success Rate", fontsize=11)

    # Mark best cell
    best = grouped.loc[grouped["success_rate"].idxmax()]
    ax.plot(best["Q_vel"], best["Q_pos"], "k*", markersize=15, label="Best")
    ax.legend(fontsize=10)

    plt.tight_layout()
    return fig


def plot_dv_heatmap(grouped, title_suffix=""):
    Q_pos_vals, Q_vel_vals, grid = build_heatmap_grid(grouped, "mean_dv")

    fig, ax = plt.subplots(figsize=(10, 8))
    im = ax.pcolormesh(
        Q_vel_vals, Q_pos_vals, grid,
        cmap="viridis_r", shading="nearest"
    )

    ax.set_xscale("log")
    ax.set_yscale("log")
    ax.set_xlabel("Q_vel", fontsize=12)
    ax.set_ylabel("Q_pos", fontsize=12)
    ax.set_title(f"Mean \u0394V (m/s){title_suffix}", fontsize=14)

    cbar = fig.colorbar(im, ax=ax)
    cbar.set_label("\u0394V (m/s)", fontsize=11)

    # Mark lowest dv cell with >80% success
    viable = grouped[grouped["success_rate"] > 0.8]
    if not viable.empty:
        best = viable.loc[viable["mean_dv"].idxmin()]
        ax.plot(best["Q_vel"], best["Q_pos"], "r*", markersize=15, label="Min \u0394V (>80% success)")
        ax.legend(fontsize=10)

    plt.tight_layout()
    return fig

def plot_initial_state_analysis(df, title_suffix=""):
    failed = df[df["success"] == 0]
    passed = df[df["success"] == 1]

    fig, axes = plt.subplots(1, 3, figsize=(15, 5))

    axes[0].scatter(passed["y0"], passed["x0"], s=1, alpha=0.3, label="Success")    
    axes[0].scatter(failed["y0"], failed["x0"], s=1, alpha=0.3, label="Fail")
    axes[0].set_xlabel("y0 (along-track)")
    axes[0].set_ylabel("x0 (radial)")
    axes[0].legend()

    axes[1].scatter(passed["y0"], passed["z0"], s=1, alpha=0.3)
    axes[1].scatter(failed["y0"], failed["z0"], s=1, alpha=0.3)
    axes[1].set_xlabel("y0")
    axes[1].set_ylabel("z0")

    # Range vs success
    passed_range = np.sqrt(passed["x0"]**2 + passed["y0"]**2 + passed["z0"]**2)
    failed_range = np.sqrt(failed["x0"]**2 + failed["y0"]**2 + failed["z0"]**2)
    axes[2].hist(passed_range, bins=30, alpha=0.5, label="Success")
    axes[2].hist(failed_range, bins=30, alpha=0.5, label="Fail")
    axes[2].set_xlabel("Initial range (m)")
    axes[2].legend()

    plt.tight_layout()

def plot_velocity_analysis(df, title_suffix=""):
    failed = df[df["success"] == 0]
    passed = df[df["success"] == 1]

    fig, axes = plt.subplots(1, 3, figsize=(15, 5))

    # Velocity magnitude
    passed_vmag = np.sqrt(passed["vx0"]**2 + passed["vy0"]**2 + passed["vz0"]**2)
    failed_vmag = np.sqrt(failed["vx0"]**2 + failed["vy0"]**2 + failed["vz0"]**2)
    axes[0].hist(passed_vmag, bins=30, alpha=0.5, label="Success")
    axes[0].hist(failed_vmag, bins=30, alpha=0.5, label="Fail")
    axes[0].set_xlabel("Initial velocity magnitude (m/s)")
    axes[0].legend()

    # vy0 specifically — along-track, toward/away from target
    axes[1].hist(passed["vy0"], bins=30, alpha=0.5, label="Success")
    axes[1].hist(failed["vy0"], bins=30, alpha=0.5, label="Fail")
    axes[1].set_xlabel("vy0 (along-track, m/s)")
    axes[1].legend()

    # Range vs velocity magnitude colored by success
    range_all = np.sqrt(df["x0"]**2 + df["y0"]**2 + df["z0"]**2)
    vmag_all = np.sqrt(df["vx0"]**2 + df["vy0"]**2 + df["vz0"]**2)
    colors = df["success"].map({1: "blue", 0: "orange"})
    axes[2].scatter(range_all, vmag_all, c=colors, s=1, alpha=0.3)
    axes[2].set_xlabel("Initial range (m)")
    axes[2].set_ylabel("Initial velocity magnitude (m/s)")

    fig.suptitle(f"Velocity Analysis{title_suffix}", fontsize=14)
    plt.tight_layout()

def print_summary(grouped):
    print("\n" + "=" * 60)
    print("SWEEP SUMMARY")
    print("=" * 60)

    best_success = grouped.loc[grouped["success_rate"].idxmax()]
    print(f"\nHighest success rate: {best_success['success_rate']:.1%}")
    print(f"  Q_pos = {best_success['Q_pos']:.2f}, Q_vel = {best_success['Q_vel']:.2f}")
    print(f"  Mean dV = {best_success['mean_dv']:.2f} m/s")

    viable = grouped[grouped["success_rate"] > 0.8]
    if not viable.empty:
        fuel_opt = viable.loc[viable["mean_dv"].idxmin()]
        print(f"\nMost fuel-efficient (>80% success): {fuel_opt['mean_dv']:.2f} m/s")
        print(f"  Q_pos = {fuel_opt['Q_pos']:.2f}, Q_vel = {fuel_opt['Q_vel']:.2f}")
        print(f"  Success rate = {fuel_opt['success_rate']:.1%}")

    # Regions with >90% success
    high_success = grouped[grouped["success_rate"] > 0.9]
    if not high_success.empty:
        print(f"\nCells with >90% success: {len(high_success)} / {len(grouped)}")
        print(f"  Q_pos range: [{high_success['Q_pos'].min():.2f}, {high_success['Q_pos'].max():.2f}]")
        print(f"  Q_vel range: [{high_success['Q_vel'].min():.2f}, {high_success['Q_vel'].max():.2f}]")
    else:
        print("\nNo cells achieved >90% success rate")

    print("=" * 60)



# -------------------------------------------------------------------
# Main
# -------------------------------------------------------------------

if __name__ == "__main__":
    filename = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_FILE
    filepath = os.path.join(CSV_DIR, filename)

    if not os.path.exists(filepath):
        print(f"File not found: {filepath}")
        sys.exit(1)

    df = load_sweep(filepath)
    
    grouped = aggregate_grid(df)

    print_summary(grouped)

    axis_name = filename.replace("_sweep.csv", "").upper()
    suffix = f" — {axis_name} Approach"

    fig1 = plot_success_rate_heatmap(grouped, suffix)
    fig2 = plot_dv_heatmap(grouped, suffix)
    fig3 = plot_initial_state_analysis(df, suffix)
    fig4 = plot_velocity_analysis(df, suffix)

    plt.show()