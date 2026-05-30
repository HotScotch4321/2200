"""
ploting test.c data and generating sample logs.

Supply real captures from the robot to replace them.

    python3 plot_test_data.py speed  speed_log.csv
    python3 plot_test_data.py line   Kp0p3.csv Kp1p0.csv Kp2p5.csv
    python3 plot_test_data.py slow   slow_log.csv

TICK_TO_MS: calibrate against a known straight run on the real robot.
"""

import sys, csv
import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl

mpl.rcParams.update({
    "font.family": "serif", "font.size": 10,
    "axes.titlesize": 11, "axes.titleweight": "bold", "axes.labelsize": 10,
    "axes.grid": True, "grid.alpha": 0.3, "grid.linewidth": 0.6,
    "legend.fontsize": 8.5, "legend.framealpha": 0.9,
    "figure.dpi": 150, "savefig.dpi": 150, "savefig.bbox": "tight",
})
C_BLUE, C_RED, C_GREEN, C_ORANGE, C_GREY = "#1f4e79", "#c00000", "#2e7d32", "#e07b00", "#7f7f7f"
TICK_TO_MS = 0.40 / 180.0


def read_csv(path):
    with open(path) as f:
        rows = list(csv.reader(f))
    head = [h.strip() for h in rows[0]]
    cols = {h: [] for h in head}
    for r in rows[1:]:
        if len(r) != len(head):
            continue
        for h, v in zip(head, r):
            cols[h].append(float(v))
    return {h: np.array(cols[h]) for h in head}


def settle_time_ms(t_ms, signal, target, tol, hold_ms=200):
    ok = np.abs(signal - target) <= tol
    step = max(1, int(round(np.median(np.diff(t_ms)))))
    hold_n = max(1, int(round(hold_ms / step)))
    for i in range(len(signal) - hold_n + 1):
        if ok[i:i + hold_n].all():
            return t_ms[i]
    return None


def overshoot_pct(signal):
    s0 = signal[0]
    if abs(s0) < 1e-6:
        return 0.0
    opp = -np.min(signal) if s0 > 0 else np.max(signal)
    return max(0.0, opp) / abs(s0) * 100.0


def plot_speed(path, out="fig_test_speed.png"):
    d = read_csv(path)
    t = d["t_ms"] / 1000.0
    fig, (ax, axe) = plt.subplots(2, 1, figsize=(9.6, 5.8), sharex=True,
                                   gridspec_kw={"height_ratios": [2.4, 1.0]})
    ax.step(t, d["target"],       where="post", color=C_GREY, ls="--", lw=1.4, label="target")
    ax.plot(t, d["actual_left"],  color=C_BLUE, lw=1.7, label="actual, left wheel")
    ax.plot(t, d["actual_right"], color=C_RED,  lw=1.5, ls="-.", label="actual, right wheel")
    ax.set_ylabel("Wheel speed (encoder ticks / 10 ms)")
    ax.set_title("Speed-loop step response: target vs measured (Kp=2.0, Ki=0.5)")
    ax.legend(loc="lower right")
    sec = ax.secondary_yaxis("right", functions=(lambda v: v * TICK_TO_MS, lambda v: v / TICK_TO_MS))
    sec.set_ylabel("approx. m/s")

    up_idx = np.where(np.diff(d["target"]) > 0)[0]
    if len(up_idx):
        k = up_idx[0] + 1
        ts = settle_time_ms(d["t_ms"][k:], d["actual_left"][k:],
                            d["target"][k], tol=0.05 * d["target"][k])
        if ts is not None:
            ax.axvline(d["t_ms"][k] / 1000.0, color=C_ORANGE, ls=":", lw=0.9)
            ax.annotate(f"settles to within 5%\nin ~{ts - d['t_ms'][k]:.0f} ms",
                        xy=(ts / 1000.0, d["target"][k]),
                        xytext=(ts / 1000.0 + 0.15, d["target"][k] - 55),
                        arrowprops=dict(arrowstyle="->", color=C_ORANGE),
                        color=C_ORANGE, fontsize=8.5)

    axe.plot(t, d["actual_left"]  - d["target"], color=C_BLUE, lw=1.5, label="left error")
    axe.plot(t, d["actual_right"] - d["target"], color=C_RED,  lw=1.3, ls="-.", label="right error")
    axe.axhline(0, color="k", lw=0.8)
    axe.set_ylabel("tracking error\n(ticks)")
    axe.set_xlabel("Time (s)")
    axe.legend(loc="upper right", ncol=2)
    fig.savefig(out); plt.close(fig)
    print("wrote", out)


def plot_line(paths, labels=None, out="fig_test_linetune.png"):
    cols = [C_RED, C_ORANGE, C_BLUE, C_GREEN]
    fig, ax = plt.subplots(figsize=(9.6, 4.8))
    band = 15
    summary = []
    for i, p in enumerate(paths):
        d = read_csv(p)
        lab = labels[i] if labels else p
        ax.plot(d["t_ms"] / 1000.0, d["position"],
                color=cols[i % len(cols)], lw=1.7, label=lab)
        summary.append((
            lab,
            settle_time_ms(d["t_ms"], d["position"], 0.0, tol=band),
            overshoot_pct(d["position"]),
            float(np.std(d["position"][d["t_ms"] > 1000])),
        ))

    ax.axhline(0, color="k", lw=0.8)
    ax.axhspan(-band, band, color=C_GREEN, alpha=0.08)
    ax.set_title("Line-following tuning: return to centre after a placed offset")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Measured line position (units)")
    ax.legend(loc="upper right")

    rows = [f"{'gain set':<16}{'settle':>8}{'over':>7}{'jitter':>8}"]
    for lab, ts, ov, jit in summary:
        st = ("%.2fs" % (ts / 1000.0)) if ts is not None else "none"
        rows.append(f"{lab[:16]:<16}{st:>8}{ov:>6.0f}%{jit:>7.1f}")
    ax.text(0.985, 0.05, "\n".join(rows), transform=ax.transAxes,
            ha="right", va="bottom", fontsize=7.4, family="monospace",
            bbox=dict(boxstyle="round", fc="white", ec=C_GREY, alpha=0.9))
    fig.savefig(out); plt.close(fig)
    print("wrote", out)


def plot_slow(path, out="fig_test_slowzone.png"):
    d = read_csv(path)
    t, state = d["t_ms"], d["state"]
    fig, ax = plt.subplots(figsize=(10.0, 4.8))

    def shade(val, col, lab):
        on = state == val
        if not on.any():
            return
        edges = np.diff(np.concatenate([[0], on.astype(int), [0]]))
        for s, e in zip(np.where(edges == 1)[0], np.where(edges == -1)[0]):
            ax.axvspan(t[s] / 1000.0, t[min(e, len(t) - 1)] / 1000.0,
                       color=col, alpha=0.10, lw=0)
        ax.axvspan(np.nan, np.nan, color=col, alpha=0.25, label=lab)

    shade(0, C_BLUE, "following")
    shade(1, C_RED,  "slow zone")
    ax.plot(t / 1000.0, d["actual_left"],  color=C_BLUE, lw=1.8, label="left wheel")
    ax.plot(t / 1000.0, d["actual_right"], color=C_RED,  lw=1.8, label="right wheel")
    ax.step(t / 1000.0, d["target"], where="post", color=C_GREY, ls="--", lw=1.2, label="target")
    sec = ax.secondary_yaxis("right", functions=(lambda v: v * TICK_TO_MS, lambda v: v / TICK_TO_MS))
    sec.set_ylabel("approx. m/s")
    ax.set_title("Slow-zone entry: wheel speed and response time after the red marker")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Wheel speed (encoder ticks / 10 ms)")

    trans = np.where(np.diff(state) == 1)[0]
    if len(trans):
        k = trans[0] + 1
        t0, slow_tgt = t[k], d["target"][k]
        ax.axvline(t0 / 1000.0, color=C_RED, lw=1.0, ls=":")
        ax.annotate("red marker sensed",
                    xy=(t0 / 1000.0, slow_tgt + 40),
                    xytext=(t0 / 1000.0 + 0.25, slow_tgt + 70),
                    arrowprops=dict(arrowstyle="->", color=C_RED),
                    color=C_RED, fontsize=8.5)
        reach = settle_time_ms(t[k:], d["actual_left"][k:], slow_tgt,
                               tol=0.10 * slow_tgt, hold_ms=150)
        if reach is not None:
            ax.axvline(reach / 1000.0, color=C_GREEN, lw=1.0, ls=":")
            ax.annotate(f"reached slow speed\n{reach - t0:.0f} ms after sensing",
                        xy=(reach / 1000.0, slow_tgt),
                        xytext=(reach / 1000.0 + 0.2, slow_tgt - 55),
                        arrowprops=dict(arrowstyle="->", color=C_GREEN),
                        color=C_GREEN, fontsize=8.5)
    ax.legend(loc="upper right", ncol=2)
    fig.savefig(out); plt.close(fig)
    print("wrote", out)


# Sample log generation using the same first-order motor model as the design study.
def _motor_track(targets, tau_s=0.06, kp=2.0, ki=0.5, noise=2.0, seed=0):
    rng = np.random.default_rng(seed)
    dt = 0.01; alpha = dt / tau_s; w = 0.0; integ = 0.0; out = []
    for tgt in targets:
        err = tgt - w
        integ = np.clip(integ + err, -500, 500)
        w += alpha * (np.clip(kp * err + ki * integ, 0, 255) - w)
        out.append(w + rng.normal(0, noise))
    return np.array(out)

def write_demo_logs():
    tgt = np.concatenate([np.full(60, 130), np.full(60, 180), np.full(60, 90)])
    aL, aR = _motor_track(tgt, seed=1), _motor_track(tgt, seed=2, tau_s=0.065)
    with open("speed_log.csv", "w", newline="") as f:
        w = csv.writer(f); w.writerow(["t_ms", "target", "actual_left", "actual_right"])
        for i in range(len(tgt)):
            w.writerow([i * 10, int(tgt[i]), int(round(aL[i])), int(round(aR[i]))])

    def line_run(Kp, Kd, seed):
        rng = np.random.default_rng(seed)
        dt = 0.01; b = 0.15; base = 180; C = 0.40 / 180; slope = 15.8; L = 0.08
        y = 4.0 / 1000.0; th = 0.0; prev = 0.0; log = []
        for _ in range(300):
            m = float(np.clip(slope * (y + L * np.sin(th)) * 1000, -700, 700)) + rng.normal(0, 1.2)
            err = -m; d = err - prev; prev = err; out = Kp * err + Kd * d
            tL = np.clip(base - out, 0, 255); tR = np.clip(base + out, 0, 255)
            v = 0.5 * C * (tL + tR); om = C * (tR - tL) / b
            th += om * dt; y += v * np.sin(th) * dt
            log.append(m)
        return np.array(log)

    for fname, args in [("line_Kp0p3.csv", (0.3, 0.5, 3)),
                        ("line_Kp1p0.csv", (1.0, 0.5, 4)),
                        ("line_Kp2p5.csv", (2.5, 0.5, 5))]:
        pos = line_run(*args)
        with open(fname, "w", newline="") as f:
            w = csv.writer(f); w.writerow(["t_ms", "position", "error", "pid_out"])
            for i, p in enumerate(pos):
                w.writerow([i * 10, int(round(p)), int(round(-p)), 0])

    n = 380
    ts = np.arange(n) * 10
    tgt3  = np.where(ts < 1200, 180, np.where(ts < 3000, 90, 130)).astype(float)
    st3   = np.where((ts >= 1200) & (ts < 3000), 1, 0)
    aL3, aR3 = _motor_track(tgt3, seed=6), _motor_track(tgt3, seed=7, tau_s=0.065)
    with open("slow_log.csv", "w", newline="") as f:
        w = csv.writer(f); w.writerow(["t_ms", "state", "target", "actual_left", "actual_right"])
        for i in range(n):
            w.writerow([i * 10, int(st3[i]), int(tgt3[i]),
                        int(round(aL3[i])), int(round(aR3[i]))])
    print("wrote sample logs")


if __name__ == "__main__":
    if len(sys.argv) == 1:
        write_demo_logs()
        plot_speed("speed_log.csv")
        plot_line(["line_Kp0p3.csv", "line_Kp1p0.csv", "line_Kp2p5.csv"],
                  labels=["Kp=0.3 (low)", "Kp=1.0 (selected)", "Kp=2.5 (high)"])
        plot_slow("slow_log.csv")
    else:
        mode, args = sys.argv[1], sys.argv[2:]
        if   mode == "speed": plot_speed(args[0])
        elif mode == "line":  plot_line(args)
        elif mode == "slow":  plot_slow(args[0])
        else: print("mode: speed | line | slow")