"""
采集串口 CSV 原始数据并绘图分析。
三种数据行:
  P,seq,ir,red,ac_ir,ac_red           — PPG 逐样本 (25 SPS)
  E,ecg_val                           — ECG 逐样本
  D,ms,ir,red,ecg_raw,ppg_hr,spo2,ecg_hr,ecg_sig,finger — 500ms 摘要

直连模式（自绘板 USB）: ECG 512 SPS, 含 P/E/D 行
SLE 模式（大板中继）:   ECG 256 SPS, 仅 E/D 行（无 P 行）

用法:  python raw_capture.py               — 直连 COM6 采集 60 秒
       python raw_capture.py 30             — 直连 COM6 采集 30 秒
       python raw_capture.py --sle 30       — SLE 模式（256 SPS）从 COM6
       python raw_capture.py --port COM7 30 — 指定端口
       python raw_capture.py plot           — 只绘图（读取已有 CSV）
"""
import serial, time, sys, os, csv

PORT     = "COM6"
BAUD     = 115200
ECG_SPS  = 512  # default: direct UART

# --sle flag: switch to 256 SPS mode
if "--sle" in sys.argv:
    ECG_SPS = 256
    sys.argv.remove("--sle")

# --port COMx 参数解析
if "--port" in sys.argv:
    idx = sys.argv.index("--port")
    if idx + 1 < len(sys.argv):
        PORT = sys.argv[idx + 1]
        sys.argv.pop(idx)  # remove --port
        sys.argv.pop(idx)  # remove COMx
PPG_FILE = os.path.join(os.path.dirname(__file__), "raw_ppg.csv")
ECG_FILE = os.path.join(os.path.dirname(__file__), "raw_ecg.csv")
SUM_FILE = os.path.join(os.path.dirname(__file__), "raw_summary.csv")
PPG_HDR  = ["seq","ir","red","ac_ir","ac_red"]
ECG_HDR  = ["ecg"]
SUM_HDR  = ["ms","ir","red","ecg_raw","ppg_hr","spo2","ecg_hr","ecg_sig","finger"]

def capture(seconds):
    sys.stdout.reconfigure(encoding='utf-8', errors='replace')
    try:
        ser = serial.Serial(PORT, BAUD, timeout=1)
    except serial.SerialException as e:
        print(f"Cannot open {PORT}: {e}")
        sys.exit(1)
    mode = "SLE relay" if ECG_SPS == 256 else "direct"
    print(f"Capture {PORT} @ {BAUD}  {seconds}s  ({mode}, ECG {ECG_SPS} SPS) ...")
    ser.reset_input_buffer()
    ppg_rows, ecg_rows, sum_rows = [], [], []
    start = time.time()
    while time.time() - start < seconds:
        try:
            raw = ser.readline()
            if not raw:
                continue
            line = raw.decode('utf-8', errors='replace').strip()
            if line.startswith("P,"):
                parts = line[2:].split(",")
                if len(parts) == len(PPG_HDR):
                    ppg_rows.append(parts)
            elif line.startswith("E,"):
                parts = line[2:].split(",")
                if len(parts) == 1:
                    ecg_rows.append(parts)
            elif line.startswith("D,"):
                parts = line[2:].split(",")
                if len(parts) == len(SUM_HDR):
                    sum_rows.append(parts)
                    print(f"\r  t={parts[0]:>7s}ms  IR={parts[1]:>6s}  RED={parts[2]:>6s}  "
                          f"HR={parts[4]:>3s}  SpO2={parts[5]:>3s}%  "
                          f"eHR={parts[6]:>3s}  eSIG={parts[7]:>3s}  "
                          f"P={len(ppg_rows)} E={len(ecg_rows)}", end="")
        except serial.SerialException as e:
            print(f"\n[ERR] {e}")
            break
    ser.close()
    for path, hdr, rows in [(PPG_FILE, PPG_HDR, ppg_rows),
                             (ECG_FILE, ECG_HDR, ecg_rows),
                             (SUM_FILE, SUM_HDR, sum_rows)]:
        with open(path, "w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(hdr)
            w.writerows(rows)
    print(f"\nDone: PPG {len(ppg_rows)}  ECG {len(ecg_rows)}  SUM {len(sum_rows)}")
    print(f"  -> {PPG_FILE}\n  -> {ECG_FILE}\n  -> {SUM_FILE}")

def plot():
    import matplotlib.pyplot as plt
    import numpy as np
    sys.stdout.reconfigure(encoding='utf-8', errors='replace')

    ppg = {h: [] for h in PPG_HDR}
    if os.path.isfile(PPG_FILE):
        with open(PPG_FILE, "r", encoding="utf-8") as f:
            for row in csv.DictReader(f):
                for h in PPG_HDR:
                    ppg[h].append(int(row[h]))
    ecg_vals = []
    if os.path.isfile(ECG_FILE):
        with open(ECG_FILE, "r", encoding="utf-8") as f:
            for row in csv.DictReader(f):
                ecg_vals.append(int(row["ecg"]))
    sm = {h: [] for h in SUM_HDR}
    if os.path.isfile(SUM_FILE):
        with open(SUM_FILE, "r", encoding="utf-8") as f:
            for row in csv.DictReader(f):
                for h in SUM_HDR:
                    sm[h].append(int(row[h]))

    np_ppg = len(ppg["seq"])
    ne = len(ecg_vals)
    ns = len(sm["ms"])

    global ECG_SPS
    if np_ppg == 0 and ne > 0 and ns > 0:
        ECG_SPS = 256

    print(f"PPG: {np_ppg}  ECG: {ne}  SUM: {ns}")

    fig, axes = plt.subplots(6, 1, figsize=(16, 16), sharex=False)
    fig.suptitle(f"Raw Waveform  (PPG {np_ppg}, ECG {ne}, SUM {ns})", fontsize=13)

    if np_ppg > 0:
        seq = np.array(ppg["seq"])
        t_ppg = (seq - seq[0]) / 25.0
        ax = axes[0]
        ax.plot(t_ppg, ppg["ir"], linewidth=0.5, color="red", label="IR")
        ax.plot(t_ppg, ppg["red"], linewidth=0.5, color="blue", label="RED")
        ax.set_ylabel("PPG raw")
        ax.set_title("PPG Raw (25 SPS)")
        ax.legend(loc="upper right", fontsize=8)
        ax.grid(True, alpha=0.3)

        ax = axes[1]
        ax.plot(t_ppg, ppg["ac_ir"], linewidth=0.5, color="red", label="AC_IR")
        ax.plot(t_ppg, ppg["ac_red"], linewidth=0.5, color="blue", label="AC_RED")
        ax.axhline(y=0, color="black", linewidth=0.5)
        ax.set_ylabel("PPG AC")
        ax.set_title("PPG AC (pulse wave)")
        ax.legend(loc="upper right", fontsize=8)
        ax.grid(True, alpha=0.3)
    elif ns > 0:
        ir_arr = np.array(sm["ir"], dtype=np.float64)
        red_arr = np.array(sm["red"], dtype=np.float64)
        t_s = np.array(sm["ms"]) / 1000.0
        t_s = t_s - t_s[0]

        ax = axes[0]
        ax.plot(t_s, ir_arr, linewidth=1.2, color="red", marker=".", markersize=3, label="IR")
        if np.any(red_arr > 0):
            ax.plot(t_s, red_arr, linewidth=1.2, color="blue", marker=".", markersize=3, label="RED")
        ax.set_ylabel("PPG raw")
        ax.set_title("PPG from Summary (2 Hz, SLE mode)")
        ax.legend(loc="upper right", fontsize=8)
        ax.grid(True, alpha=0.3)

        ax = axes[1]
        if len(ir_arr) > 5:
            win = min(5, len(ir_arr) // 2)
            kernel = np.ones(win) / win
            ir_dc = np.convolve(ir_arr, kernel, mode='same')
            ir_ac = ir_arr - ir_dc
            ir_ac[:win] = 0
            ir_ac[-win:] = 0
            ax.plot(t_s, ir_ac, linewidth=1.2, color="red", marker=".", markersize=3, label="IR AC")
            if np.any(red_arr > 0):
                red_dc = np.convolve(red_arr, kernel, mode='same')
                red_ac = red_arr - red_dc
                red_ac[:win] = 0
                red_ac[-win:] = 0
                ax.plot(t_s, red_ac, linewidth=1.2, color="blue", marker=".", markersize=3, label="RED AC")
            ax.axhline(y=0, color="black", linewidth=0.5)
        ax.set_ylabel("PPG AC")
        ax.set_title("PPG AC from Summary (SLE mode)")
        ax.legend(loc="upper right", fontsize=8)
        ax.grid(True, alpha=0.3)

    if ne > 0:
        t_ecg = np.arange(ne) / float(ECG_SPS)
        ax = axes[2]
        ax.plot(t_ecg, ecg_vals, linewidth=0.4, color="green")
        ax.set_ylabel("ECG filtered")
        ax.set_title(f"ECG ({ECG_SPS} SPS, {ne} pts)")
        ax.grid(True, alpha=0.3)

    if ns > 0:
        t_sum = np.array(sm["ms"]) / 1000.0
        t_sum -= t_sum[0]

        ax = axes[3]
        ax.plot(t_sum, sm["ppg_hr"], label="PPG HR", linewidth=1.2, color="red",
                marker=".", markersize=3)
        ax.plot(t_sum, sm["ecg_hr"], label="ECG HR", linewidth=1.2, color="green",
                marker=".", markersize=3)
        ax.set_ylabel("HR (bpm)")
        ax.set_ylim(30, 200)
        ax.legend(loc="upper right", fontsize=8)
        ax.grid(True, alpha=0.3)

        ax = axes[4]
        ax.plot(t_sum, sm["spo2"], linewidth=1.2, color="purple",
                marker=".", markersize=3)
        ax.set_ylabel("SpO2 (%)")
        ax.set_ylim(80, 105)
        ax.grid(True, alpha=0.3)

        ax = axes[5]
        sig = np.array(sm["ecg_sig"])
        ax.fill_between(t_sum, 0, sig, alpha=0.4, color="orange", label="ECG poor_signal")
        finger = np.array(sm["finger"])
        ax.plot(t_sum, finger * 200, linewidth=1, color="blue", label="PPG finger x200")
        ax.set_ylabel("Quality")
        ax.set_xlabel("Time (s)")
        ax.legend(loc="upper right", fontsize=8)
        ax.grid(True, alpha=0.3)

    plt.tight_layout()
    png = os.path.join(os.path.dirname(__file__), "raw_plot.png")
    plt.savefig(png, dpi=150)
    print(f"Plot saved: {png}")
    plt.show()

if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "plot":
        plot()
    else:
        sec = int(sys.argv[1]) if len(sys.argv) > 1 else 60
        capture(sec)
        plot()
