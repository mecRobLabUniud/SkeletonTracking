#!/usr/bin/env python3
"""
Plot all data inside /home/lab/SkeletonTracking/logs using matplotlib with a
LaTeX interpreter, and save every figure as a PDF into
/home/lab/SkeletonTracking/scripts/data.

The log folder contains one sub-folder per task (e.g. SSMPFL, SSMPFLescape),
each holding XML files produced by the C++ logger:

    q.xml, qd.xml, qdd.xml   -> joint position / velocity / acceleration (7 joints)
    p.xml, pd.xml            -> end-effector Cartesian position / velocity (x, y, z)
    distance.xml             -> robot-human distance + indices / closest points
    metrics.txt              -> task summary metrics

Usage (inside the docker container):
    python3 /home/lab/SkeletonTracking/scripts/plot_logs.py
"""

import os
import glob
import xml.etree.ElementTree as ET

import numpy as np
import matplotlib
matplotlib.use("Agg")
matplotlib.rcParams["text.usetex"] = True
matplotlib.rcParams["font.size"] = 11
matplotlib.rcParams["axes.labelsize"] = 11
matplotlib.rcParams["axes.titlesize"] = 12
matplotlib.rcParams["legend.fontsize"] = 10
matplotlib.rcParams["xtick.labelsize"] = 9
matplotlib.rcParams["ytick.labelsize"] = 9
import matplotlib.pyplot as plt


# ---------------------------------------------------------------------------
# Paths
# ---------------------------------------------------------------------------
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, os.pardir))
LOGS_DIR = os.path.join(PROJECT_ROOT, "logs")
OUT_DIR = os.path.join(PROJECT_ROOT, "scripts", "data")


# ---------------------------------------------------------------------------
# XML parsing helpers
# ---------------------------------------------------------------------------
def _clean_number(text):
    """Strip trailing apostrophes / whitespace and convert to float."""
    if text is None:
        return np.nan
    text = text.strip().rstrip("'").strip()
    try:
        return float(text)
    except ValueError:
        return np.nan


def recover_root(path):
    """
    Parse an XML file even if it was truncated mid-write (missing closing
    tags). Extracts every complete ``<keypoint>...</keypoint>`` block and
    re-wraps them in a synthetic root element.

    Returns the root element, or None if no complete keypoint could be found.
    """
    try:
        with open(path, "r") as f:
            text = f.read()
    except OSError:
        return None

    first = text.find("<keypoint")
    last = text.rfind("</keypoint>")
    if first == -1 or last == -1:
        return None

    last_end = last + len("</keypoint>")
    inner = text[first:last_end]
    wrapped = f"<root>{inner}</root>"
    try:
        return ET.fromstring(wrapped)
    except ET.ParseError:
        return None


def parse_generic_xml(path, value_tags):
    """
    Parse an XML file whose structure is::

        <root>
          <keypoint time='t'>
            <tag id='0'>value</tag>
            <tag id='1'>value</tag>
            ...
          </keypoint>
        </root>

    Truncated files are recovered via :func:`recover_root`.

    ``value_tags`` is the tag name to look for inside each keypoint
    (e.g. ``'joint'`` for q.xml, ``'point'`` for p.xml).

    Returns
    -------
    t : (N,) array of times
    values : (N, M) array of values (M = number of distinct ids found)
    ids : list of column id strings
    Returns None if the file cannot be parsed.
    """
    root = recover_root(path)
    if root is None:
        print(f"  [skip] {os.path.basename(path)}: no complete keypoints")
        return None

    records = []
    for kp in root.findall("keypoint"):
        t = float(kp.get("time", np.nan))
        vals = {}
        for tag in kp.findall(value_tags):
            idx = tag.get("id", None)
            vals[idx] = _clean_number(tag.text)
        records.append((t, vals))

    # Determine column order from the union of all ids (numeric sort).
    all_ids = sorted(
        {idx for _, vals in records for idx in vals.keys()},
        key=lambda x: (0, int(x)) if (x is not None and str(x).lstrip("-").isdigit()) else (1, str(x)),
    )

    t = np.array([r[0] for r in records])
    values = np.full((len(records), len(all_ids)), np.nan)
    for row, (_, vals) in enumerate(records):
        for col, idx in enumerate(all_ids):
            values[row, col] = vals.get(idx, np.nan)

    return t, values, all_ids


def parse_distance_xml(path):
    """
    Parse distance.xml::

        <distance>
          <keypoint time='t'>
            <length>...</length>
            <h_index>...</h_index>
            <r_index>...</r_index>
            <c_h><point id='x'>..</point>...</c_h>
            <c_r>...</c_r>
          </keypoint>
        </distance>

    Returns dict with time, length, h_index, r_index, c_h (N,3), c_r (N,3).
    Returns None if the file cannot be parsed.
    """
    root = recover_root(path)
    if root is None:
        print(f"  [skip] {os.path.basename(path)}: no complete keypoints")
        return None

    t, length, h_index, r_index = [], [], [], []
    c_h, c_r = [], []

    for kp in root:
        t.append(float(kp.get("time", np.nan)))
        length.append(_clean_number(kp.findtext("length")))
        h_index.append(_clean_number(kp.findtext("h_index")))
        r_index.append(_clean_number(kp.findtext("r_index")))

        ch = kp.find("c_h")
        cr = kp.find("c_r")
        c_h.append([_clean_number(pt.text) for pt in ch.findall("point")]
                   if ch is not None else [np.nan] * 3)
        c_r.append([_clean_number(pt.text) for pt in cr.findall("point")]
                   if cr is not None else [np.nan] * 3)

    return {
        "t": np.array(t),
        "length": np.array(length),
        "h_index": np.array(h_index),
        "r_index": np.array(r_index),
        "c_h": np.array(c_h),
        "c_r": np.array(c_r),
    }


# ---------------------------------------------------------------------------
# Plotting
# ---------------------------------------------------------------------------
def save_fig(fig, out_path):
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    fig.savefig(out_path, bbox_inches="tight")
    plt.close(fig)


def plot_vector_vs_time(t, values, labels, ylabel, title, out_path):
    """Multiple curves vs time on a single axis."""
    fig, ax = plt.subplots(figsize=(7, 4))
    for col in range(values.shape[1]):
        ax.plot(t, values[:, col], label=fr"${labels[col]}$", linewidth=1.2)
    ax.set_xlabel(r"time $[s]$")
    ax.set_ylabel(ylabel)
    ax.set_title(title)
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best", ncol=min(len(labels), 4), framealpha=0.9)
    save_fig(fig, out_path)


def plot_subplots_vs_time(t, values, labels, ylabel, title, out_path):
    """One subplot per curve vs time."""
    n = values.shape[1]
    ncols = 2
    nrows = int(np.ceil(n / ncols))
    fig, axes = plt.subplots(nrows, ncols, figsize=(8, 2.6 * nrows), sharex=True)
    if n == 1:
        axes = np.array([axes])
    axes = axes.ravel()
    for col in range(n):
        ax = axes[col]
        ax.plot(t, values[:, col], linewidth=1.2, color="C0")
        ax.set_ylabel(rf"${labels[col]}$")
        ax.grid(True, alpha=0.3)
        ax.set_title(rf"${labels[col]}$ vs time")
    # Hide unused subplots.
    for col in range(n, len(axes)):
        axes[col].axis("off")
    axes[-1].set_xlabel(r"time $[s]$")
    fig.suptitle(title, fontsize=12)
    fig.tight_layout(rect=[0, 0.03, 1, 0.95])
    save_fig(fig, out_path)


def process_log_folder(folder_path, out_dir):
    """Plot every XML file inside one log sub-folder."""
    name = os.path.basename(folder_path)
    print(f"Processing log folder: {name}")

    xml_files = sorted(glob.glob(os.path.join(folder_path, "*.xml")))

    for path in xml_files:
        base = os.path.splitext(os.path.basename(path))[0]

        if base == "q":
            res = parse_generic_xml(path, "joint")
            if res is None:
                continue
            t, values, ids = res
            labels = [f"q_{{{i}}}" for i in ids]
            plot_vector_vs_time(
                t, values, labels,
                r"joint position $[rad]$",
                rf"{name} -- joint positions $q$",
                os.path.join(out_dir, f"{name}_q.pdf"),
            )
            plot_subplots_vs_time(
                t, values, labels,
                r"$[rad]$",
                rf"{name} -- joint positions $q$ (per joint)",
                os.path.join(out_dir, f"{name}_q_subplots.pdf"),
            )

        elif base == "qd":
            res = parse_generic_xml(path, "joint")
            if res is None:
                continue
            t, values, ids = res
            labels = [f"\dot{{q}}_{{{i}}}" for i in ids]
            plot_vector_vs_time(
                t, values, labels,
                r"joint velocity $[rad/s]$",
                rf"{name} -- joint velocities $\dot{{q}}$",
                os.path.join(out_dir, f"{name}_qd.pdf"),
            )
            plot_subplots_vs_time(
                t, values, labels,
                r"$[rad/s]$",
                rf"{name} -- joint velocities $\dot{{q}}$ (per joint)",
                os.path.join(out_dir, f"{name}_qd_subplots.pdf"),
            )

        elif base == "qdd":
            res = parse_generic_xml(path, "joint")
            if res is None:
                continue
            t, values, ids = res
            labels = [f"\ddot{{q}}_{{{i}}}" for i in ids]
            plot_vector_vs_time(
                t, values, labels,
                r"joint acceleration $[rad/s^2]$",
                rf"{name} -- joint accelerations $\ddot{{q}}$",
                os.path.join(out_dir, f"{name}_qdd.pdf"),
            )
            plot_subplots_vs_time(
                t, values, labels,
                r"$[rad/s^2]$",
                rf"{name} -- joint accelerations $\ddot{{q}}$ (per joint)",
                os.path.join(out_dir, f"{name}_qdd_subplots.pdf"),
            )

        elif base == "p":
            res = parse_generic_xml(path, "point")
            if res is None:
                continue
            t, values, ids = res
            labels = ["x", "y", "z"]
            plot_vector_vs_time(
                t, values, labels,
                r"position $[m]$",
                rf"{name} -- end-effector position $p$",
                os.path.join(out_dir, f"{name}_p.pdf"),
            )

        elif base == "pd":
            res = parse_generic_xml(path, "point")
            if res is None:
                continue
            t, values, ids = res
            labels = ["\dot{x}", "\dot{y}", "\dot{z}"]
            plot_vector_vs_time(
                t, values, labels,
                r"velocity $[m/s]$",
                rf"{name} -- end-effector velocity $\dot{{p}}$",
                os.path.join(out_dir, f"{name}_pd.pdf"),
            )

        elif base == "distance":
            data = parse_distance_xml(path)
            if data is None:
                continue
            t = data["t"]

            # Distance length vs time.
            fig, ax = plt.subplots(figsize=(7, 4))
            ax.plot(t, data["length"], linewidth=1.2, color="tab:red")
            ax.set_xlabel(r"time $[s]$")
            ax.set_ylabel(r"robot--human distance $[m]$")
            ax.set_title(rf"{name} -- robot--human distance $\ell$")
            ax.grid(True, alpha=0.3)
            save_fig(fig, os.path.join(out_dir, f"{name}_distance.pdf"))

            # Obstacle indices vs time.
            fig, ax = plt.subplots(figsize=(7, 4))
            ax.plot(t, data["h_index"], label=r"$h$ (human)", linewidth=1.2, color="tab:blue")
            ax.plot(t, data["r_index"], label=r"$r$ (robot)", linewidth=1.2, color="tab:green")
            ax.set_xlabel(r"time $[s]$")
            ax.set_ylabel("closest-point index")
            ax.set_title(rf"{name} -- closest-point indices")
            ax.grid(True, alpha=0.3)
            ax.legend(framealpha=0.9)
            save_fig(fig, os.path.join(out_dir, f"{name}_distance_indices.pdf"))

            # Closest-point coordinates (human c_h and robot c_r) vs time.
            fig, axes = plt.subplots(2, 3, figsize=(9, 5), sharex=True)
            names_h = ["x", "y", "z"]
            names_r = ["x", "y", "z"]
            for col in range(3):
                axes[0, col].plot(t, data["c_h"][:, col], linewidth=1.2, color="tab:red")
                axes[0, col].set_ylabel(rf"$c_{{h,{names_h[col]}}}$ $[m]$")
                axes[0, col].grid(True, alpha=0.3)
                axes[1, col].plot(t, data["c_r"][:, col], linewidth=1.2, color="tab:green")
                axes[1, col].set_ylabel(rf"$c_{{r,{names_r[col]}}}$ $[m]$")
                axes[1, col].grid(True, alpha=0.3)
                axes[1, col].set_xlabel(r"time $[s]$")
            fig.suptitle(rf"{name} -- closest-point coordinates", fontsize=12)
            fig.tight_layout(rect=[0, 0.03, 1, 0.95])
            save_fig(fig, os.path.join(out_dir, f"{name}_distance_points.pdf"))

    # Metrics summary text file -> small table figure.
    metrics_path = os.path.join(folder_path, "metrics.txt")
    if os.path.exists(metrics_path):
        metrics = {}
        with open(metrics_path, "r") as f:
            for line in f:
                if ":" in line:
                    key, val = line.split(":", 1)
                    metrics[key.strip()] = val.strip()

        fig, ax = plt.subplots(figsize=(6, 2.6))
        ax.axis("off")
        rows = [[key, val] for key, val in metrics.items()]
        table = ax.table(
            cellText=rows,
            colLabels=["Metric", "Value"],
            loc="center",
            cellLoc="left",
        )
        table.auto_set_font_size(False)
        table.set_fontsize(10)
        table.scale(1, 1.8)
        ax.set_title(rf"{name} -- task metrics", fontsize=12)
        save_fig(fig, os.path.join(out_dir, f"{name}_metrics.pdf"))


def main():
    os.makedirs(OUT_DIR, exist_ok=True)

    if not os.path.isdir(LOGS_DIR):
        print(f"Logs directory not found: {LOGS_DIR}")
        return

    subfolders = sorted(
        d for d in glob.glob(os.path.join(LOGS_DIR, "*")) if os.path.isdir(d)
    )
    if not subfolders:
        print(f"No log sub-folders found in {LOGS_DIR}")
        return

    for folder in subfolders:
        process_log_folder(folder, OUT_DIR)

    print(f"\nAll plots saved as PDF to: {OUT_DIR}")


if __name__ == "__main__":
    main()
