#!/usr/bin/env python3
"""Ve do thi so sanh 4 phuong phap uoc luong goc tu file CSV.

Nhan CSV tu hai nguon, tu nhan dang:
  - host_test/out.csv : co ground truth (cot truth_roll/truth_pitch/truth_yaw)
  - CSV tu board that : khong co ground truth, chi ve 4 duong de so sanh

Dung:
    tools/.venv/bin/python tools/plot_csv.py host_test/out.csv
    tools/.venv/bin/python tools/plot_csv.py board.csv -o hinh.png
"""
import argparse
import csv
import math
import sys

import matplotlib
matplotlib.use("Agg")            # khong can X server - chay duoc qua SSH
import matplotlib.pyplot as plt


def read_csv(path):
    """Doc CSV, bo qua moi dong khong phai du lieu.

    Can thiet vi CSV tu board di chung UART voi log cua ESP_LOG, nen file se co
    lan cac dong log. Bo dong thay vi bao loi la dung: mat vai mau khong sao,
    bat nguoi dung phai loc tay thi moi la van de.
    """
    rows, header, skipped = [], None, 0
    with open(path, newline="") as f:
        for raw in f:
            line = raw.strip()
            if not line:
                continue
            if line.startswith("#"):
                if header is None:
                    header = next(csv.reader([line.lstrip("#")]))
                continue
            parts = next(csv.reader([line]))
            if header is None:
                # Dong tieu de khong co dau '#' (truong hop out.csv cua host_test)
                if any(c.isalpha() for c in parts[0]):
                    header = parts
                    continue
                skipped += 1
                continue
            if len(parts) != len(header):
                skipped += 1
                continue
            try:
                rows.append([float(x) for x in parts])
            except ValueError:
                skipped += 1

    if header is None or not rows:
        sys.exit(f"Khong doc duoc du lieu tu {path} (bo qua {skipped} dong)")
    if skipped:
        print(f"  Bo qua {skipped} dong khong phai du lieu (log lan vao UART)")

    cols = {name: [r[i] for r in rows] for i, name in enumerate(header)}
    return cols, len(rows)


def rmse(est, truth):
    """RMSE co xu ly goc tuan hoan: 179 va -179 lech 2 do, khong phai 358."""
    n = min(len(est), len(truth))
    if n == 0:
        return float("nan")
    acc = 0.0
    for i in range(n):
        e = (est[i] - truth[i] + 180.0) % 360.0 - 180.0
        acc += e * e
    return math.sqrt(acc / n)


# Bang mau: mot mau cho moi phuong phap, dung nhat quan tren moi hinh.
STYLE = {
    "truth": dict(color="#111111", lw=2.2, ls="-",  label="Goc that (ground truth)", zorder=5),
    "acc":   dict(color="#d1495b", lw=0.7, ls="-",  alpha=0.55, label="Chi gia toc ke", zorder=2),
    "gyro":  dict(color="#edae49", lw=1.1, ls="--", label="Chi con quay (troi)",      zorder=3),
    "comp":  dict(color="#00798c", lw=1.1, ls="-",  alpha=0.85, label="Complementary", zorder=4),
    "kf":    dict(color="#2e86ab", lw=1.8, ls="-",  label="KALMAN",                   zorder=6),
}


def plot_axis(ax, t, cols, axis, has_truth):
    """Ve mot truc (roll hoac pitch) voi day du cac duong co trong file."""
    if has_truth and f"truth_{axis}" in cols:
        ax.plot(t, cols[f"truth_{axis}"], **STYLE["truth"])
    for key, prefix in (("acc", "acc"), ("gyro", "gyro"), ("comp", "comp")):
        name = f"{axis}_{prefix}"
        if name in cols:
            ax.plot(t, cols[name], **STYLE[key])
    if f"{axis}_kf" in cols:
        ax.plot(t, cols[f"{axis}_kf"], **STYLE["kf"])

    ax.set_ylabel(f"{axis} [do]")
    ax.grid(True, alpha=0.25, ls=":")
    ax.legend(loc="upper right", fontsize=8, ncol=2, framealpha=0.9)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv", help="host_test/out.csv hoac CSV ghi tu board")
    ap.add_argument("-o", "--out", default=None, help="file PNG ra (mac dinh: <csv>.png)")
    ap.add_argument("--dpi", type=int, default=140)
    args = ap.parse_args()

    cols, n = read_csv(args.csv)
    print(f"Doc {n} mau, {len(cols)} cot tu {args.csv}")

    # Truc thoi gian: host_test dung 't' (giay), firmware dung 't_ms'.
    if "t" in cols:
        t = cols["t"]
    elif "t_ms" in cols:
        t = [v / 1000.0 for v in cols["t_ms"]]
    else:
        t = list(range(n))

    has_truth = "truth_roll" in cols

    # --- Bang RMSE, chi khi co ground truth ---
    if has_truth:
        print("\n  Phuong phap              RMSE roll    RMSE pitch")
        print("  ----------------------  ----------    ----------")
        for label, suffix in (("Chi gia toc ke", "acc"), ("Chi con quay", "gyro"),
                              ("Complementary", "comp"), ("KALMAN", "kf")):
            r = rmse(cols.get(f"roll_{suffix}", []),  cols["truth_roll"])
            p = rmse(cols.get(f"pitch_{suffix}", []), cols["truth_pitch"])
            print(f"  {label:22}  {r:8.3f} do    {p:8.3f} do")

    n_panels = 4 if "acc_norm" in cols else 2
    fig, axes = plt.subplots(n_panels, 1, figsize=(13, 3.0 * n_panels), sharex=True)

    plot_axis(axes[0], t, cols, "roll", has_truth)
    plot_axis(axes[1], t, cols, "pitch", has_truth)

    title = "MPU6050 - so sanh cac phuong phap uoc luong goc"
    if has_truth:
        rk = rmse(cols["roll_kf"], cols["truth_roll"])
        ra = rmse(cols["roll_acc"], cols["truth_roll"])
        title += f"   (RMSE roll: Kalman {rk:.2f} do  vs  gia toc ke {ra:.2f} do)"
    axes[0].set_title(title, fontsize=11)

    if n_panels == 4:
        # Panel 3: muc nhieu gia toc va phan ung cua R thich nghi.
        ax = axes[2]
        ax.plot(t, cols["acc_norm"], color="#5f0f40", lw=0.8, label="|a| [g]")
        ax.axhline(1.0, color="#999", ls=":", lw=1)
        ax.set_ylabel("|a| [g]")
        ax.grid(True, alpha=0.25, ls=":")
        ax.legend(loc="upper left", fontsize=8)

        if "R_used" in cols:
            ax2 = ax.twinx()
            ax2.plot(t, cols["R_used"], color="#f18f01", lw=1.0, label="R dang dung")
            ax2.set_ylabel("R [do^2]")
            ax2.legend(loc="upper right", fontsize=8)

        # Panel 4: bias con quay do Kalman uoc luong + trang thai dung yen.
        ax = axes[3]
        for key, color, lab in (("bias_x", "#d1495b", "bias X"),
                                ("bias_y", "#00798c", "bias Y"),
                                ("bias_z", "#edae49", "bias Z")):
            if key in cols:
                ax.plot(t, cols[key], color=color, lw=1.2, label=lab)
        if "is_static" in cols:
            ax.fill_between(t, 0, 1, where=[v > 0.5 for v in cols["is_static"]],
                            transform=ax.get_xaxis_transform(),
                            color="#2e86ab", alpha=0.10, label="dung yen (ZRU)")
        ax.set_ylabel("bias [do/s]")
        ax.grid(True, alpha=0.25, ls=":")
        ax.legend(loc="upper right", fontsize=8, ncol=4)

    axes[-1].set_xlabel("Thoi gian [s]")
    fig.tight_layout()

    out = args.out or (args.csv.rsplit(".", 1)[0] + ".png")
    fig.savefig(out, dpi=args.dpi)
    print(f"\nDa ghi {out}")


if __name__ == "__main__":
    main()
