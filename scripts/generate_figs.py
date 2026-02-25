import math
import os
import matplotlib.pyplot as plt

FIG_DIR = os.path.join(os.path.dirname(__file__), "..", "docs", "figures")
os.makedirs(FIG_DIR, exist_ok=True)


def speed_scaling():
    d_stop = 15.0
    d_slow = 120.0
    ds = [i for i in range(0, 201)]
    s = [max(0.0, min(1.0, (d - d_stop) / (d_slow - d_stop))) for d in ds]
    plt.figure(figsize=(6.4, 2.6))
    plt.plot(ds, s, lw=2)
    plt.xlabel("d (distance)")
    plt.ylabel("s(d)")
    plt.ylim(0, 1.05)
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig(os.path.join(FIG_DIR, "speed_scaling.png"), dpi=200)
    plt.close()


def repulsive_field():
    k = 6000.0
    d0 = 160.0
    vmax = 600.0
    ds = [i for i in range(5, 121)]
    v = []
    for d in ds:
        if d >= d0:
            v.append(0.0)
            continue
        mag = k * (1.0 / d - 1.0 / d0) / (d * d)
        v.append(min(vmax, max(0.0, mag)))
    plt.figure(figsize=(6.4, 2.6))
    plt.plot(ds, v, lw=2)
    plt.xlabel("d (distance)")
    plt.ylabel("|v_rep|")
    plt.ylim(0, vmax)
    plt.xlim(5, 120)
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig(os.path.join(FIG_DIR, "repulsion.png"), dpi=200)
    plt.close()


def damped_barrier():
    d_stop = 15.0
    d_slow = 120.0
    ds = [i for i in range(0, 201)]
    s = [max(0.0, min(1.0, (d - d_stop) / (d_slow - d_stop))) for d in ds]
    plt.figure(figsize=(6.4, 2.6))
    plt.plot(ds, s, lw=2)
    plt.xlabel("d (distance)")
    plt.ylabel("normal scale")
    plt.ylim(0, 1.05)
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig(os.path.join(FIG_DIR, "damped_barrier.png"), dpi=200)
    plt.close()


def normal_projection():
    xs = [x / 50.0 for x in range(-60, 61)]
    ys = [0.0 if x < 0 else x for x in xs]
    plt.figure(figsize=(6.4, 2.6))
    plt.plot(xs, ys, lw=2)
    plt.xlabel("v_cmd · n")
    plt.ylabel("v_safe · n")
    plt.xlim(-1.2, 1.2)
    plt.ylim(-0.1, 1.2)
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig(os.path.join(FIG_DIR, "normal_projection.png"), dpi=200)
    plt.close()


def main():
    speed_scaling()
    repulsive_field()
    damped_barrier()
    normal_projection()
    print(f"Wrote figures to {FIG_DIR}")


if __name__ == "__main__":
    main()
