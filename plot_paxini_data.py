#!/usr/bin/env python3
"""
PaXini Dual Tactile Sensor Data Visualizer & Analysis Tool
==========================================================
Visualizes and analyzes exported CSV recordings from the Unified Control System
for the PaXini DP-S2015-Elite tactile sensors (Left and Right gripper fingers).

Key Features:
1. Automatic Session Detection:
   Auto-detects the latest synchronized recording session (L and R sensors)
   in the workspace or accepts custom file paths.
2. Synchronized 3D Force & Grip Balance Analysis (Figure 1):
   - Resultant 3D forces (Fx, Fy, Fz) and total magnitude |F|.
   - Grip symmetry comparison: Left Fz vs. Right Fz.
   - Tangential shear force (F_shear) and friction utilization ratio (mu = Ft/Fz).
   - Contact area (active taxels) and peak taxel normal force.
3. Spatial Contact & CoP Trajectory Analysis (Figure 2):
   - 2D Center of Pressure (CoP_x vs. CoP_y) trajectories on the sensor surface.
   - Force vector angles: Shear direction and surface tilt.
   - Slip friction phase plot (mu vs. Fz) with incipient slip threshold.
4. 2D 52-Taxel Spatial Pressure Heatmaps (Figure 3):
   - Reconstructs spatial taxel pressure distribution on physical coordinates
     (S2015_ELITE_XYZ) at the peak grasp moment for both fingers.
   - Center of Pressure (+) marker and tangential shear vector arrow.
5. Automated Console Diagnostic Report & High-DPI Plot Export.

Usage:
    # Auto-detect and plot the latest session:
    python plot_paxini_data.py

    # Plot specific CSV files:
    python plot_paxini_data.py paxini_20261011_151951_L_COM8.csv paxini_20261011_151951_R_COM13.csv

    # Save plots without opening interactive windows:
    python plot_paxini_data.py --no-show --output-dir ./plots
"""

from __future__ import annotations

import argparse
import glob
import math
import os
import re
import sys
from typing import Dict, List, Optional, Tuple

import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
import numpy as np

# ---------------------------------------------------------------------------
# S2015-Elite 52-Taxel Spatial Coordinates (mm)
# Columns: X, Y, Z. Row k corresponds to taxel #(k+1).
# ---------------------------------------------------------------------------
S2015_ELITE_XYZ = np.array([
    [-7.45866948, -0.80442889, 1.9377763], [-7.24032101, 1.9370259, 1.93780601],
    [-7.00988064, 5.22921165, 1.93775712], [-6.82610763, 7.97333403, 1.93769891],
    [-6.53146658, 11.26026387, 1.93782951], [-6.00006622, -1.35478789, 4.53417757],
    [-6.00372647, 1.50179184, 4.67021242], [-5.94055243, 4.93336235, 4.69448457],
    [-5.81660022, 7.79060929, 4.65440797], [-6.07106495, 13.96973406, 1.93782184],
    [-5.51333141, 11.20680746, 4.536848], [-5.0017979, 14.00765355, 4.28734202],
    [-5.09707632, 16.57086028, 1.93780027], [-4.40866763, 16.2194301, 3.70766158],
    [-3.1037709, -1.55570508, 5.41014857], [-3.15461212, 1.5038129, 5.90611644],
    [-3.20348468, 5.19993, 6.32908896], [-3.21601164, 8.3044369, 6.53483487],
    [-3.1409429, 11.42030336, 6.47498956], [-2.83527189, 15.06167934, 5.87056449],
    [-2.42756984, 17.70908967, 4.36265159], [-2.71496388, 18.81872693, 1.93777956],
    [0.0, -1.55808447, 5.4116888], [0.0, 1.60830268, 5.95092329],
    [0.0, 5.43294958, 6.43279476], [0.0, 8.63424733, 6.69381112],
    [0.0, 12.48579856, 6.62785389], [0.0, 15.63993516, 6.05667345],
    [0.0, 18.02972832, 4.50323706], [0.0, 19.32616275, 1.93777338],
    [3.15461212, 1.5038129, 5.90611644], [3.1409429, 11.42030336, 6.47498956],
    [2.83527189, 15.06167934, 5.87056449], [2.42756984, 17.70908967, 4.36265159],
    [3.1037709, -1.55570508, 5.41014857], [3.20348468, 5.19993, 6.32908896],
    [3.21601164, 8.3044369, 6.53483487], [2.71496388, 18.81872693, 1.93777956],
    [4.40866763, 16.2194301, 3.70766158], [5.0017979, 14.00765355, 4.28734202],
    [5.09707632, 16.57086028, 1.93780027], [6.00006622, -1.35478789, 4.53417757],
    [6.00372647, 1.50179184, 4.67021242], [5.94055243, 4.93336235, 4.69448457],
    [5.81660022, 7.79060929, 4.65440797], [5.51333141, 11.20680746, 4.536848],
    [6.53146658, 11.26026387, 1.93782951], [6.07106495, 13.96973406, 1.93782184],
    [7.24032101, 1.9370259, 1.93780601], [7.00988064, 5.22921165, 1.93775712],
    [6.82610763, 7.97333403, 1.93769891], [7.45866948, -0.80442889, 1.9377763],
], dtype=np.float64)


# ===========================================================================
# Data Representation Class
# ===========================================================================
class SensorDataset:
    """Holds time series and 52-taxel spatial data parsed from a PaXini CSV file."""

    def __init__(self, filepath: str) -> None:
        self.filepath = os.path.abspath(filepath)
        self.filename = os.path.basename(filepath)
        self.tag = "Unknown"
        self.port = "Unknown"
        self.meta = ""

        # Parse header & comment
        with open(self.filepath, "r", encoding="utf-8", errors="ignore") as f:
            raw_lines = [line.strip() for line in f if line.strip()]

        data_lines = []
        for line in raw_lines:
            if line.startswith("#"):
                self.meta = line.lstrip("#").strip()
                # Extract sensor tag and port
                m_tag = re.search(r"sensor=([A-Za-z0-9_-]+)", self.meta)
                if m_tag:
                    self.tag = m_tag.group(1).upper()
                m_port = re.search(r"port=([A-Za-z0-9_-]+)", self.meta)
                if m_port:
                    self.port = m_port.group(1)
            else:
                data_lines.append(line)

        # Fallback to filename tag parsing
        if self.tag == "Unknown":
            if "_L_" in self.filename or "_Left_" in self.filename:
                self.tag = "L"
            elif "_R_" in self.filename or "_Right_" in self.filename:
                self.tag = "R"

        if not data_lines:
            raise ValueError(f"CSV file '{filepath}' is empty or contains no data lines.")

        self.header = data_lines[0].split(",")
        # Parse data array using numpy
        self.raw_data = np.genfromtxt(data_lines[1:], delimiter=",")
        if self.raw_data.ndim == 1:
            self.raw_data = self.raw_data.reshape(1, -1)

        self.num_samples = self.raw_data.shape[0]
        self.num_cols = self.raw_data.shape[1]

        # Extract time
        self.t = self.raw_data[:, 0]
        self.epoch = self.raw_data[:, 1]

        # Extract resultant forces & metrics
        self.fx = self.raw_data[:, 2]
        self.fy = self.raw_data[:, 3]
        self.fz = self.raw_data[:, 4]
        self.f_mag = self.raw_data[:, 5]
        self.f_shear = self.raw_data[:, 6]
        self.mu = self.raw_data[:, 7]
        self.cop_x = self.raw_data[:, 8]
        self.cop_y = self.raw_data[:, 9]
        self.shear_dir = self.raw_data[:, 10]
        self.tilt = self.raw_data[:, 11]
        self.peak_taxel = self.raw_data[:, 12]
        self.peak_fz = self.raw_data[:, 13]
        self.contact_taxels = self.raw_data[:, 14]

        # Extract 52 distributed taxels: (N, 52, 3) where dim 2 is [Fx, Fy, Fz]
        if self.num_cols >= 15 + 52 * 3:
            taxel_block = self.raw_data[:, 15:15 + 52 * 3]
            self.taxels = taxel_block.reshape(self.num_samples, 52, 3)
        else:
            self.taxels = None

        # Statistics
        self.duration = float(self.t[-1] - self.t[0]) if self.num_samples > 1 else 0.0
        dt_mean = float(np.mean(np.diff(self.t))) if self.num_samples > 1 else 0.02
        self.sampling_rate = (1.0 / dt_mean) if dt_mean > 0 else 0.0
        self.peak_fz_overall = float(np.nanmax(self.fz))
        self.peak_fmag_overall = float(np.nanmax(self.f_mag))
        self.peak_fshear_overall = float(np.nanmax(self.f_shear))
        self.max_contact_taxels = int(np.nanmax(self.contact_taxels))
        self.peak_frame_idx = int(np.nanargmax(self.fz))
        self.peak_time = float(self.t[self.peak_frame_idx])

    @property
    def label(self) -> str:
        name = "Left Sensor" if self.tag == "L" else ("Right Sensor" if self.tag == "R" else f"Sensor {self.tag}")
        return f"{name} ({self.port})"


# ===========================================================================
# Session Discovery Helper
# ===========================================================================
def find_paxini_sessions(search_dirs: List[str]) -> Dict[str, Dict[str, str]]:
    """Scan directories for paxini_*.csv recordings and group by session timestamp."""
    sessions: Dict[str, Dict[str, str]] = {}
    pattern = re.compile(r"paxini_(\d{8}_\d{6})_([LR])_([A-Za-z0-9_-]+)\.csv")

    for s_dir in search_dirs:
        if not os.path.isdir(s_dir):
            continue
        for fname in os.listdir(s_dir):
            m = pattern.match(fname)
            if m:
                stamp, tag, port = m.groups()
                fpath = os.path.join(s_dir, fname)
                if stamp not in sessions:
                    sessions[stamp] = {}
                sessions[stamp][tag] = fpath

    return sessions


# ===========================================================================
# Plotting Routines
# ===========================================================================
def plot_forces_and_grip(ds_left: Optional[SensorDataset],
                        ds_right: Optional[SensorDataset],
                        save_path: Optional[str] = None) -> plt.Figure:
    """Figure 1: Synchronized 3D Forces, Normal Grip Balance, and Contact Area."""
    fig = plt.figure(figsize=(15, 11))
    fig.canvas.manager.set_window_title("PaXini Tactile - Force & Grip Analysis")
    gs = GridSpec(3, 2, figure=fig, hspace=0.35, wspace=0.22)

    # Subplot 1: Left Sensor Resultant Forces
    ax1 = fig.add_subplot(gs[0, 0])
    if ds_left:
        ax1.plot(ds_left.t, ds_left.fx, label="Fx (lateral)", color="#ff4d4d", lw=1.5)
        ax1.plot(ds_left.t, ds_left.fy, label="Fy (longitudinal)", color="#2ecc71", lw=1.5)
        ax1.plot(ds_left.t, ds_left.fz, label="Fz (normal)", color="#3498db", lw=2.0)
        ax1.plot(ds_left.t, ds_left.f_mag, label="|F| total", color="#f39c12", lw=1.2, ls="--")
        ax1.set_title(f"Left Jaw: 3D Forces [{ds_left.port}]", fontweight="bold")
    else:
        ax1.text(0.5, 0.5, "Left Sensor Data Not Available", ha="center", va="center")
    ax1.set_ylabel("Force (N)")
    ax1.set_xlabel("Relative Time (s)")
    ax1.grid(True, linestyle="--", alpha=0.5)
    ax1.legend(loc="upper left", framealpha=0.8, fontsize=9)

    # Subplot 2: Right Sensor Resultant Forces
    ax2 = fig.add_subplot(gs[0, 1])
    if ds_right:
        ax2.plot(ds_right.t, ds_right.fx, label="Fx (lateral)", color="#ff4d4d", lw=1.5)
        ax2.plot(ds_right.t, ds_right.fy, label="Fy (longitudinal)", color="#2ecc71", lw=1.5)
        ax2.plot(ds_right.t, ds_right.fz, label="Fz (normal)", color="#3498db", lw=2.0)
        ax2.plot(ds_right.t, ds_right.f_mag, label="|F| total", color="#f39c12", lw=1.2, ls="--")
        ax2.set_title(f"Right Jaw: 3D Forces [{ds_right.port}]", fontweight="bold")
    else:
        ax2.text(0.5, 0.5, "Right Sensor Data Not Available", ha="center", va="center")
    ax2.set_ylabel("Force (N)")
    ax2.set_xlabel("Relative Time (s)")
    ax2.grid(True, linestyle="--", alpha=0.5)
    ax2.legend(loc="upper left", framealpha=0.8, fontsize=9)

    # Subplot 3: Grip Normal Force Comparison (Fz_Left vs. Fz_Right)
    ax3 = fig.add_subplot(gs[1, 0])
    if ds_left and ds_right:
        n_common = min(len(ds_left.t), len(ds_right.t))
        t_common = ds_left.t[:n_common]
        fz_l_common = ds_left.fz[:n_common]
        fz_r_common = ds_right.fz[:n_common]
        ax3.plot(ds_left.t, ds_left.fz, label="Left Fz (Normal)", color="#1f77b4", lw=2.0)
        ax3.plot(ds_right.t, ds_right.fz, label="Right Fz (Normal)", color="#e67e22", lw=2.0)
        ax3.fill_between(t_common, 0, np.minimum(fz_l_common, fz_r_common),
                         color="#27ae60", alpha=0.15, label="Shared Contact Force")
        # Grip Balance annotation
        diff = np.abs(fz_l_common - fz_r_common)
        max_diff = float(np.nanmax(diff))
        ax3.set_title(f"Grip Normal Force Symmetry (Max |Fz_L - Fz_R| = {max_diff:.2f} N)", fontweight="bold")
    elif ds_left:
        ax3.plot(ds_left.t, ds_left.fz, label="Left Fz", color="#1f77b4", lw=2.0)
        ax3.set_title("Normal Force Fz (Single Sensor)", fontweight="bold")
    elif ds_right:
        ax3.plot(ds_right.t, ds_right.fz, label="Right Fz", color="#e67e22", lw=2.0)
        ax3.set_title("Normal Force Fz (Single Sensor)", fontweight="bold")
    ax3.set_ylabel("Normal Force Fz (N)")
    ax3.set_xlabel("Relative Time (s)")
    ax3.grid(True, linestyle="--", alpha=0.5)
    ax3.legend(loc="upper left", framealpha=0.8, fontsize=9)

    # Subplot 4: Tangential Shear Force & Friction Ratio (mu)
    ax4 = fig.add_subplot(gs[1, 1])
    if ds_left:
        ax4.plot(ds_left.t, ds_left.f_shear, label="Left F_shear (Ft)", color="#8e44ad", lw=1.8)
    if ds_right:
        ax4.plot(ds_right.t, ds_right.f_shear, label="Right F_shear (Ft)", color="#e74c3c", lw=1.8, ls="-.")
    ax4.set_ylabel("Shear Force Ft (N)", color="#8e44ad")
    ax4.tick_params(axis="y", labelcolor="#8e44ad")
    ax4.grid(True, linestyle="--", alpha=0.5)

    # Secondary Y axis for friction coefficient mu
    ax4_mu = ax4.twinx()
    if ds_left:
        ax4_mu.plot(ds_left.t, ds_left.mu, label=r"Left $\mu = Ft/Fz$", color="#16a085", lw=1.2, ls=":")
    if ds_right:
        ax4_mu.plot(ds_right.t, ds_right.mu, label=r"Right $\mu = Ft/Fz$", color="#2980b9", lw=1.2, ls=":")
    ax4_mu.axhline(0.65, color="red", ls="--", lw=1.0, alpha=0.7, label=r"Slip Threshold $\mu=0.65$")
    ax4_mu.set_ylabel(r"Friction Ratio $\mu$", color="#16a085")
    ax4_mu.tick_params(axis="y", labelcolor="#16a085")
    ax4_mu.set_ylim(0, 1.5)
    ax4.set_title("Tangential Shear Force & Friction Utilization", fontweight="bold")
    ax4.set_xlabel("Relative Time (s)")

    lines1, labels1 = ax4.get_legend_handles_labels()
    lines2, labels2 = ax4_mu.get_legend_handles_labels()
    ax4.legend(lines1 + lines2, labels1 + labels2, loc="upper right", framealpha=0.8, fontsize=8)

    # Subplot 5: Contact Size (Active Contact Taxels)
    ax5 = fig.add_subplot(gs[2, 0])
    if ds_left:
        ax5.plot(ds_left.t, ds_left.contact_taxels, label="Left Finger Active Taxels", color="#1f77b4", lw=1.8)
    if ds_right:
        ax5.plot(ds_right.t, ds_right.contact_taxels, label="Right Finger Active Taxels", color="#e67e22", lw=1.8, ls="--")
    ax5.set_title("Contact Area: Active Taxels Count (Threshold >= 0.1 N)", fontweight="bold")
    ax5.set_ylabel("Number of Contact Taxels")
    ax5.set_xlabel("Relative Time (s)")
    ax5.set_ylim(-1, 53)
    ax5.grid(True, linestyle="--", alpha=0.5)
    ax5.legend(loc="upper left", framealpha=0.8, fontsize=9)

    # Subplot 6: Peak Taxel Normal Force
    ax6 = fig.add_subplot(gs[2, 1])
    if ds_left:
        ax6.plot(ds_left.t, ds_left.peak_fz, label="Left Peak Taxel Fz", color="#2c3e50", lw=1.8)
    if ds_right:
        ax6.plot(ds_right.t, ds_right.peak_fz, label="Right Peak Taxel Fz", color="#c0392b", lw=1.8, ls="--")
    ax6.set_title("Maximum Single-Taxel Normal Force (Peak Pressure Point)", fontweight="bold")
    ax6.set_ylabel("Peak Taxel Force (N)")
    ax6.set_xlabel("Relative Time (s)")
    ax6.grid(True, linestyle="--", alpha=0.5)
    ax6.legend(loc="upper left", framealpha=0.8, fontsize=9)

    fig.suptitle("PaXini S2015-Elite: Dual-Sensor Force & Grip Dynamic Analysis", fontsize=15, fontweight="bold", y=0.99)
    if save_path:
        fig.savefig(save_path, dpi=300, bbox_inches="tight")
        print(f"  [Saved Figure 1]: {save_path}")
    return fig


def plot_cop_and_angles(ds_left: Optional[SensorDataset],
                        ds_right: Optional[SensorDataset],
                        save_path: Optional[str] = None) -> plt.Figure:
    """Figure 2: Center of Pressure (CoP) Surface Trajectory and Vector Angles."""
    fig = plt.figure(figsize=(15, 10))
    fig.canvas.manager.set_window_title("PaXini Tactile - CoP & Force Angles")
    gs = GridSpec(2, 2, figure=fig, hspace=0.32, wspace=0.22)

    # Subplot 1: Left CoP Trajectory on Sensor Fingerpad
    ax1 = fig.add_subplot(gs[0, 0])
    ax1.scatter(S2015_ELITE_XYZ[:, 0], S2015_ELITE_XYZ[:, 1], c="#dcdde1", s=70, edgecolors="#718093", label="Taxels (52)")
    if ds_left:
        valid = ~np.isnan(ds_left.cop_x) & ~np.isnan(ds_left.cop_y)
        if np.any(valid):
            sc1 = ax1.scatter(ds_left.cop_x[valid], ds_left.cop_y[valid], c=ds_left.t[valid],
                              cmap="plasma", s=25, alpha=0.85, label="Left CoP Path")
            cbar1 = plt.colorbar(sc1, ax=ax1, pad=0.02)
            cbar1.set_label("Time (s)", fontsize=8)
            # Annotate Start & End
            v_idx = np.where(valid)[0]
            ax1.plot(ds_left.cop_x[v_idx[0]], ds_left.cop_y[v_idx[0]], "go", markersize=8, label="Touch Start")
            ax1.plot(ds_left.cop_x[v_idx[-1]], ds_left.cop_y[v_idx[-1]], "rs", markersize=8, label="Touch End")
        ax1.set_title(f"Left Finger: Center of Pressure [{ds_left.port}]", fontweight="bold")
    ax1.set_xlabel("CoP X (mm)")
    ax1.set_ylabel("CoP Y (mm)")
    ax1.set_aspect("equal", "datalim")
    ax1.grid(True, linestyle="--", alpha=0.5)
    ax1.legend(loc="lower left", fontsize=8)

    # Subplot 2: Right CoP Trajectory on Sensor Fingerpad
    ax2 = fig.add_subplot(gs[0, 1])
    ax2.scatter(S2015_ELITE_XYZ[:, 0], S2015_ELITE_XYZ[:, 1], c="#dcdde1", s=70, edgecolors="#718093", label="Taxels (52)")
    if ds_right:
        valid = ~np.isnan(ds_right.cop_x) & ~np.isnan(ds_right.cop_y)
        if np.any(valid):
            sc2 = ax2.scatter(ds_right.cop_x[valid], ds_right.cop_y[valid], c=ds_right.t[valid],
                              cmap="viridis", s=25, alpha=0.85, label="Right CoP Path")
            cbar2 = plt.colorbar(sc2, ax=ax2, pad=0.02)
            cbar2.set_label("Time (s)", fontsize=8)
            v_idx = np.where(valid)[0]
            ax2.plot(ds_right.cop_x[v_idx[0]], ds_right.cop_y[v_idx[0]], "go", markersize=8, label="Touch Start")
            ax2.plot(ds_right.cop_x[v_idx[-1]], ds_right.cop_y[v_idx[-1]], "rs", markersize=8, label="Touch End")
        ax2.set_title(f"Right Finger: Center of Pressure [{ds_right.port}]", fontweight="bold")
    ax2.set_xlabel("CoP X (mm)")
    ax2.set_ylabel("CoP Y (mm)")
    ax2.set_aspect("equal", "datalim")
    ax2.grid(True, linestyle="--", alpha=0.5)
    ax2.legend(loc="lower left", fontsize=8)

    # Subplot 3: Shear Direction Angle
    ax3 = fig.add_subplot(gs[1, 0])
    if ds_left:
        ax3.plot(ds_left.t, ds_left.shear_dir, label="Left Shear Direction", color="#8e44ad", lw=1.5)
    if ds_right:
        ax3.plot(ds_right.t, ds_right.shear_dir, label="Right Shear Direction", color="#e67e22", lw=1.5, ls="--")
    ax3.set_title(r"Tangential Shear Direction Angle ($\theta = \mathrm{atan2}(Fy, Fx)$)", fontweight="bold")
    ax3.set_ylabel("Angle (deg)")
    ax3.set_xlabel("Relative Time (s)")
    ax3.set_ylim(-185, 185)
    ax3.grid(True, linestyle="--", alpha=0.5)
    ax3.legend(loc="upper right", fontsize=9)

    # Subplot 4: Surface Tilt Angle
    ax4 = fig.add_subplot(gs[1, 1])
    if ds_left:
        ax4.plot(ds_left.t, ds_left.tilt, label="Left Force Tilt from Normal", color="#16a085", lw=1.5)
    if ds_right:
        ax4.plot(ds_right.t, ds_right.tilt, label="Right Force Tilt from Normal", color="#d35400", lw=1.5, ls="--")
    ax4.set_title(r"Force Vector Tilt Angle ($\psi = \mathrm{atan2}(F_{shear}, Fz)$)", fontweight="bold")
    ax4.set_ylabel("Tilt Angle (deg)")
    ax4.set_xlabel("Relative Time (s)")
    ax4.grid(True, linestyle="--", alpha=0.5)
    ax4.legend(loc="upper right", fontsize=9)

    fig.suptitle("PaXini S2015-Elite: Spatial Center of Pressure & Force Orientation", fontsize=15, fontweight="bold", y=0.99)
    if save_path:
        fig.savefig(save_path, dpi=300, bbox_inches="tight")
        print(f"  [Saved Figure 2]: {save_path}")
    return fig


def plot_spatial_taxel_heatmaps(ds_left: Optional[SensorDataset],
                               ds_right: Optional[SensorDataset],
                               time_s: Optional[float] = None,
                               save_path: Optional[str] = None) -> plt.Figure:
    """Figure 3: Spatial 2D 52-Taxel Pressure Distribution Heatmaps at Peak Grasp."""
    fig, (ax_l, ax_r) = plt.subplots(1, 2, figsize=(14, 7))
    fig.canvas.manager.set_window_title("PaXini Tactile - 2D Spatial Taxel Heatmaps")

    # Determine reference frame
    target_ds = ds_left or ds_right
    if not target_ds:
        return fig

    if time_s is not None:
        frame_idx = int(np.argmin(np.abs(target_ds.t - time_s)))
    else:
        frame_idx = target_ds.peak_frame_idx

    actual_time = target_ds.t[frame_idx]
    idx_l = int(np.argmin(np.abs(ds_left.t - actual_time))) if ds_left else 0
    idx_r = int(np.argmin(np.abs(ds_right.t - actual_time))) if ds_right else 0

    # Global max force for shared color scaling
    f_max_scale = 0.5
    if ds_left and ds_left.taxels is not None:
        f_max_scale = max(f_max_scale, float(np.nanmax(ds_left.taxels[idx_l, :, 2])))
    if ds_right and ds_right.taxels is not None:
        f_max_scale = max(f_max_scale, float(np.nanmax(ds_right.taxels[idx_r, :, 2])))

    # Plot Left Sensor Heatmap
    if ds_left and ds_left.taxels is not None:
        fz_tax = ds_left.taxels[idx_l, :, 2]
        sc_l = ax_l.scatter(
            S2015_ELITE_XYZ[:, 0], S2015_ELITE_XYZ[:, 1],
            c=fz_tax, cmap="inferno", s=180, edgecolors="#2c3e50", lw=1.2,
            vmin=0.0, vmax=f_max_scale
        )
        # Taxel index numbers
        for i, (tx, ty) in enumerate(S2015_ELITE_XYZ[:, :2]):
            ax_l.text(tx, ty, str(i + 1), color="white" if fz_tax[i] > f_max_scale * 0.45 else "black",
                      fontsize=6.5, ha="center", va="center", fontweight="bold")
        # Center of Pressure (+)
        cx, cy = ds_left.cop_x[idx_l], ds_left.cop_y[idx_l]
        if not math.isnan(cx) and not math.isnan(cy):
            ax_l.scatter(cx, cy, s=200, marker="+", c="#00ffff", lw=3.0, label="CoP Center")
            # Shear force arrow
            fx, fy = ds_left.fx[idx_l], ds_left.fy[idx_l]
            scale = 2.5  # mm per Newton
            ax_l.arrow(cx, cy, fx * scale, fy * scale, color="#2ecc71", width=0.3, head_width=1.0,
                       length_includes_head=True, label="Shear Force Vector")

        ax_l.set_title(f"Left Finger [{ds_left.port}]\nFz = {ds_left.fz[idx_l]:.2f} N | Active = {int(ds_left.contact_taxels[idx_l])}",
                       fontweight="bold")
        ax_l.set_xlabel("X (mm)")
        ax_l.set_ylabel("Y (mm)")
        ax_l.set_aspect("equal", "datalim")
        ax_l.grid(True, linestyle="--", alpha=0.3)
        ax_l.legend(loc="lower left", fontsize=8)
    else:
        ax_l.text(0.5, 0.5, "No Left Taxel Array Data", ha="center", va="center")

    # Plot Right Sensor Heatmap
    if ds_right and ds_right.taxels is not None:
        fz_tax_r = ds_right.taxels[idx_r, :, 2]
        sc_r = ax_r.scatter(
            S2015_ELITE_XYZ[:, 0], S2015_ELITE_XYZ[:, 1],
            c=fz_tax_r, cmap="inferno", s=180, edgecolors="#2c3e50", lw=1.2,
            vmin=0.0, vmax=f_max_scale
        )
        for i, (tx, ty) in enumerate(S2015_ELITE_XYZ[:, :2]):
            ax_r.text(tx, ty, str(i + 1), color="white" if fz_tax_r[i] > f_max_scale * 0.45 else "black",
                      fontsize=6.5, ha="center", va="center", fontweight="bold")
        cx_r, cy_r = ds_right.cop_x[idx_r], ds_right.cop_y[idx_r]
        if not math.isnan(cx_r) and not math.isnan(cy_r):
            ax_r.scatter(cx_r, cy_r, s=200, marker="+", c="#00ffff", lw=3.0, label="CoP Center")
            fx_r, fy_r = ds_right.fx[idx_r], ds_right.fy[idx_r]
            scale = 2.5
            ax_r.arrow(cx_r, cy_r, fx_r * scale, fy_r * scale, color="#2ecc71", width=0.3, head_width=1.0,
                       length_includes_head=True, label="Shear Force Vector")

        ax_r.set_title(f"Right Finger [{ds_right.port}]\nFz = {ds_right.fz[idx_r]:.2f} N | Active = {int(ds_right.contact_taxels[idx_r])}",
                       fontweight="bold")
        ax_r.set_xlabel("X (mm)")
        ax_r.set_ylabel("Y (mm)")
        ax_r.set_aspect("equal", "datalim")
        ax_r.grid(True, linestyle="--", alpha=0.3)
        ax_r.legend(loc="lower left", fontsize=8)
    else:
        ax_r.text(0.5, 0.5, "No Right Taxel Array Data", ha="center", va="center")

    # Add shared colorbar
    cbar = fig.colorbar(sc_l if 'sc_l' in locals() else sc_r, ax=[ax_l, ax_r], orientation="horizontal",
                        fraction=0.06, pad=0.12)
    cbar.set_label("Normal Force per Taxel Fz (N)", fontsize=11, fontweight="bold")

    fig.suptitle(f"PaXini S2015-Elite: 2D Spatial Taxel Pressure Distribution at Peak Moment (t = {actual_time:.2f} s)",
                 fontsize=14, fontweight="bold", y=0.98)
    if save_path:
        fig.savefig(save_path, dpi=300, bbox_inches="tight")
        print(f"  [Saved Figure 3]: {save_path}")
    return fig


# ===========================================================================
# Diagnostic Summary Report
# ===========================================================================
def print_summary_report(ds_left: Optional[SensorDataset], ds_right: Optional[SensorDataset]) -> None:
    """Print an ASCII experimental summary table to the console."""
    print("\n" + "=" * 76)
    print(" PAXINI DUAL TACTILE SENSOR - EXPERIMENTAL SUMMARY REPORT")
    print("=" * 76)

    def print_sensor_metrics(name: str, ds: Optional[SensorDataset]):
        print(f"\n--- {name} ---")
        if not ds:
            print("  Status: Not recorded in this session.")
            return
        print(f"  File                  : {ds.filename}")
        print(f"  Hardware Port         : {ds.port}")
        print(f"  Duration              : {ds.duration:.2f} s ({ds.num_samples} frames @ ~{ds.sampling_rate:.1f} Hz)")
        print(f"  Peak Normal Force Fz  : {ds.peak_fz_overall:.2f} N (at t = {ds.peak_time:.2f} s)")
        print(f"  Peak Total Force |F|  : {ds.peak_fmag_overall:.2f} N")
        print(f"  Peak Tangential Ft    : {ds.peak_fshear_overall:.2f} N")
        print(f"  Max Contact Taxels    : {ds.max_contact_taxels} / 52 taxels")
        # Peak taxel index
        pk_idx = int(ds.peak_taxel[ds.peak_frame_idx]) if not math.isnan(ds.peak_taxel[ds.peak_frame_idx]) else 0
        print(f"  Primary Contact Point : Taxel #{pk_idx} (CoP: x={ds.cop_x[ds.peak_frame_idx]:.2f} mm, y={ds.cop_y[ds.peak_frame_idx]:.2f} mm)")

    print_sensor_metrics("LEFT GRIPPER FINGER", ds_left)
    print_sensor_metrics("RIGHT GRIPPER FINGER", ds_right)

    if ds_left and ds_right:
        print("\n--- GRASP SYMMETRY & COUPLING ---")
        fz_l = ds_left.peak_fz_overall
        fz_r = ds_right.peak_fz_overall
        ratio = (fz_l / fz_r) if fz_r > 0 else 0.0
        symmetry = (1.0 - abs(fz_l - fz_r) / max(fz_l, fz_r, 1e-3)) * 100.0
        print(f"  Left vs Right Peak Fz : {fz_l:.2f} N vs {fz_r:.2f} N")
        print(f"  Grip Balance Ratio    : {ratio:.2f} (Symmetry: {symmetry:.1f}%)")
        max_mu_l = np.nanmax(ds_left.mu)
        max_mu_r = np.nanmax(ds_right.mu)
        print(f"  Peak Friction Ratio   : Left mu = {max_mu_l:.2f}, Right mu = {max_mu_r:.2f} (Threshold = 0.65)")

    print("=" * 76 + "\n")


# ===========================================================================
# Main Execution Entry Point
# ===========================================================================
def main() -> None:
    parser = argparse.ArgumentParser(
        description="Plot and analyze exported CSV tactile recordings from the Unified Control System."
    )
    parser.add_argument("files", nargs="*", help="Optional path to one or two PaXini CSV files to plot.")
    parser.add_argument("--dir", default=".", help="Directory to search for recordings (default: current directory).")
    parser.add_argument("--stamp", default="", help="Specific session timestamp to plot (e.g., 20261011_151951).")
    parser.add_argument("--left", default="", help="Explicit path to Left sensor CSV file.")
    parser.add_argument("--right", default="", help="Explicit path to Right sensor CSV file.")
    parser.add_argument("--output-dir", default="./plots", help="Directory where plot figures will be saved.")
    parser.add_argument("--no-show", action="store_true", help="Do not display interactive matplotlib GUI windows.")
    parser.add_argument("--time", type=float, default=None, help="Specific time (s) to render the 2D taxel heatmaps.")

    args = parser.parse_args()

    # Determine files to load
    left_path = args.left
    right_path = args.right

    # 1. From positional arguments
    if args.files:
        for f in args.files:
            if not os.path.isfile(f):
                print(f"[Error]: File '{f}' does not exist.")
                sys.exit(1)
            if "_L_" in f or "_Left_" in f or (not left_path and not right_path):
                left_path = f
            elif "_R_" in f or "_Right_" in f:
                right_path = f
            elif left_path and not right_path:
                right_path = f

    # 2. From auto-discovery
    if not left_path and not right_path:
        search_dirs = [args.dir, os.path.join(args.dir, "Unified_Control"), "."]
        sessions = find_paxini_sessions(search_dirs)
        if not sessions:
            # Fallback: scan for any paxini_*.csv
            csv_matches = glob.glob(os.path.join(args.dir, "paxini_*.csv"))
            if csv_matches:
                print(f"[Notice]: Found raw CSV files: {csv_matches}")
                for cm in csv_matches:
                    if "_L_" in cm:
                        left_path = cm
                    elif "_R_" in cm:
                        right_path = cm
                    elif not left_path:
                        left_path = cm
            else:
                print(f"[Error]: No PaXini recording files found in '{args.dir}'.")
                print("Make sure you have exported sensor files from the main program.")
                sys.exit(1)
        else:
            # Select specific stamp or latest
            if args.stamp and args.stamp in sessions:
                selected_stamp = args.stamp
            else:
                selected_stamp = sorted(sessions.keys())[-1]

            sess = sessions[selected_stamp]
            left_path = sess.get("L", "")
            right_path = sess.get("R", "")
            print(f"[Auto-Detected Session]: Timestamp {selected_stamp}")
            if left_path:
                print(f"  -> Left Sensor : {os.path.basename(left_path)}")
            if right_path:
                print(f"  -> Right Sensor: {os.path.basename(right_path)}")

    # Load datasets
    ds_left = SensorDataset(left_path) if left_path and os.path.isfile(left_path) else None
    ds_right = SensorDataset(right_path) if right_path and os.path.isfile(right_path) else None

    if not ds_left and not ds_right:
        print("[Error]: Could not load any valid sensor dataset.")
        sys.exit(1)

    # Print Diagnostic Summary
    print_summary_report(ds_left, ds_right)

    # Create output directory for saving figures
    os.makedirs(args.output_dir, exist_ok=True)
    stamp_tag = ""
    if ds_left:
        m = re.search(r"paxini_(\d{8}_\d{6})", ds_left.filename)
        if m:
            stamp_tag = m.group(1) + "_"

    f1_path = os.path.join(args.output_dir, f"{stamp_tag}fig1_forces_and_grip.png")
    f2_path = os.path.join(args.output_dir, f"{stamp_tag}fig2_cop_and_angles.png")
    f3_path = os.path.join(args.output_dir, f"{stamp_tag}fig3_peak_taxel_heatmaps.png")

    # Generate Figures
    print("[Generating Figures]:")
    plot_forces_and_grip(ds_left, ds_right, save_path=f1_path)
    plot_cop_and_angles(ds_left, ds_right, save_path=f2_path)
    plot_spatial_taxel_heatmaps(ds_left, ds_right, time_s=args.time, save_path=f3_path)

    print(f"\n[Done]: All figures successfully generated and saved to '{args.output_dir}'.")
    if not args.no_show:
        print("[Displaying Interactive Plots]: Close the plot windows to exit.")
        plt.show()


if __name__ == "__main__":
    main()
