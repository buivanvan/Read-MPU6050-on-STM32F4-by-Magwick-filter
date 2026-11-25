"""Plot STM32 MPU6050 telemetry from a USB-to-UART adapter."""

import argparse
from collections import deque

import matplotlib.pyplot as plt
import serial


def parse_sample(line):
    """Return timer ticks, filter mode and values in g, rad/s and degrees."""
    try:
        ticks, mode, *values = map(int, line.split(b","))
    except ValueError:
        return None
    if mode not in (0, 1) or len(values) != 6 + 3 * mode or not 0 <= ticks < 2**32:
        return None
    if any(abs(value) > 2**31 for value in values):
        return None
    scaled = [value / 1000 for value in values[:6]]
    scaled.extend(value / 100 for value in values[6:])
    return ticks, mode, scaled


def make_axes(fig, mode):
    fig.clear()
    axes = fig.subplots(2 + mode, 1, sharex=True)
    groups = [("Gia tốc (g)", "XYZ"), ("Tốc độ góc (rad/s)", "XYZ")]
    if mode:
        groups.append(("Góc (°)", ("Roll", "Pitch", "Yaw")))
    lines = []
    for ax, (label, names) in zip(axes, groups):
        ax.set_ylabel(label)
        ax.grid(alpha=0.3)
        for name in names:
            lines.extend(ax.plot([], [], label=name))
        ax.legend(loc="upper right", ncol=3)
    axes[-1].set_xlabel("Thời gian từ mẫu đầu (s)")
    return axes, lines


def plot_stream(port):
    fig = plt.figure(figsize=(10, 7), constrained_layout=True)
    fig.suptitle(f"MPU6050 · {port.port} · Đang chờ mẫu")
    plt.show(block=False)
    pending = bytearray()
    history = deque(maxlen=1000)
    active_mode = previous = None
    elapsed = 0.0
    while plt.fignum_exists(fig.number):
        pending.extend(port.read(min(port.in_waiting, 8192)))
        frames = pending.split(b"\n")
        pending = frames.pop()
        if len(pending) > 112:
            pending.clear()
        updated = False
        for frame in frames:
            sample = parse_sample(frame)
            if sample is None:
                continue
            ticks, mode, values = sample
            # Unsigned subtraction also works across a TIM2 wrap.
            delta = 0 if previous is None else (ticks - previous) % 2**32
            if mode != active_mode or delta > 2**31:
                history.clear()
                elapsed = 0.0
                delta = 0
                axes, lines = make_axes(fig, mode)
                active_mode = mode
                fig.suptitle(f"MPU6050 · {port.port} · use_filter={mode}")
            previous = ticks
            elapsed += delta / 1_000_000
            history.append((elapsed, *values))
            while history[0][0] < elapsed - 10:
                history.popleft()
            updated = True
        if updated:
            times, *channels = zip(*history)
            for line, channel in zip(lines, channels):
                line.set_data(times, channel)
            for ax in axes:
                ax.relim()
                ax.autoscale_view(scalex=False)
            axes[-1].set_xlim(max(0, elapsed - 10), max(10, elapsed))
            fig.canvas.draw_idle()
        plt.pause(0.05)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("port", help="COM3, /dev/ttyUSB0, ...")
    args = parser.parse_args()
    try:
        with serial.Serial(args.port, 115200, timeout=0) as port:
            plot_stream(port)
    except serial.SerialException as exc:
        parser.exit(1, f"Lỗi UART ({args.port}): {exc}\n")
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
