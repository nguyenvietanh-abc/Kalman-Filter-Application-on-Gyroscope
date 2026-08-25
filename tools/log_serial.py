#!/usr/bin/env python3
"""Doc CSV tu ESP32 qua UART, ghi ra file va (tuy chon) ve realtime.

Dung:
    # chi ghi file
    tools/.venv/bin/python tools/log_serial.py -p /dev/ttyUSB0 -o board.csv

    # ghi file + ve realtime cua so 20 s
    tools/.venv/bin/python tools/log_serial.py -p /dev/ttyUSB0 -o board.csv --live

    # ghi dung 60 giay roi thoat
    tools/.venv/bin/python tools/log_serial.py -p /dev/ttyUSB0 -o board.csv -d 60

Neu khong biet cong nao, chay khong tham so -p, script se liet ke cac cong tim thay.
Sau khi ghi xong, ve do thi bang:
    tools/.venv/bin/python tools/plot_csv.py board.csv
"""
import argparse
import sys
import time
from collections import deque

import serial
import serial.tools.list_ports

# Thu tu cot phai khop chinh xac tieu de firmware in ra trong main.c.
COLUMNS = [
    "t_ms", "roll_acc", "pitch_acc", "roll_gyro", "pitch_gyro",
    "roll_comp", "pitch_comp", "roll_kf", "pitch_kf", "yaw_kf",
    "bias_x", "bias_y", "bias_z", "acc_norm", "R_used", "is_static",
]


def list_ports():
    # Linux liet ke ca /dev/ttyS0..S31 - cac UART tren chipset, gan nhu luon
    # khong co gi cam vao. Chi thiet bi USB moi co VID/PID, nen day la tieu chi
    # phan biet dung: in ra 32 cong ao chi lam nguoi dung roi.
    ports = [p for p in serial.tools.list_ports.comports() if p.vid is not None]

    if not ports:
        print("Khong tim thay cong USB-serial nao.")
        print("Kiem tra:")
        print("  - Cap USB da cam chua, va la cap co day du lieu (khong phai cap chi sac)")
        print("  - Driver: ESP32 DevKit V1 dung CP2102 hoac CH340")
        print("      dmesg | tail            -> xem kernel co nhan thiet bi khong")
        print("      lsusb                   -> xem thiet bi co hien khong")
        print("  - Quyen truy cap: nguoi dung phai o group 'dialout'")
        print("      groups | grep dialout")
        return []

    print("Cac cong USB-serial tim thay:")
    for p in ports:
        print(f"  {p.device:16} {p.description}  [{p.vid:04x}:{p.pid:04x}]")
    return ports


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("-p", "--port", default=None, help="vd /dev/ttyUSB0")
    ap.add_argument("-b", "--baud", type=int, default=115200,
                    help="phai khop CONFIG_ESP_CONSOLE_UART_BAUDRATE (mac dinh 115200)")
    ap.add_argument("-o", "--out", default="board.csv")
    ap.add_argument("-d", "--duration", type=float, default=0.0,
                    help="giay; 0 = chay den khi Ctrl-C")
    ap.add_argument("--live", action="store_true", help="ve do thi realtime")
    ap.add_argument("--window", type=float, default=20.0, help="be rong cua so ve, giay")
    args = ap.parse_args()

    if not args.port:
        list_ports()
        sys.exit("\nChon mot cong roi chay lai voi -p <cong>")

    try:
        ser = serial.Serial(args.port, args.baud, timeout=1)
    except serial.SerialException as e:
        print(f"Khong mo duoc {args.port}: {e}")
        if "Permission denied" in str(e):
            print("\nThieu quyen. Them nguoi dung vao group dialout roi dang xuat/dang nhap lai:")
            print("  sudo usermod -aG dialout $USER")
        sys.exit(1)

    print(f"Mo {args.port} @ {args.baud} baud. Ghi vao {args.out}. Ctrl-C de dung.")

    fig = ax = lines = None
    hist = None
    if args.live:
        import matplotlib.pyplot as plt
        plt.ion()
        fig, ax = plt.subplots(figsize=(12, 5))
        # deque co maxlen: tu bo mau cu, khong phinh bo nho khi chay lau
        cap = int(args.window * 60)      # du cho 50 Hz co du phong
        hist = {k: deque(maxlen=cap) for k in ("t", "acc", "comp", "kf")}
        lines = {
            "acc":  ax.plot([], [], color="#d1495b", lw=0.7, alpha=0.6, label="chi gia toc ke")[0],
            "comp": ax.plot([], [], color="#00798c", lw=1.0, label="complementary")[0],
            "kf":   ax.plot([], [], color="#2e86ab", lw=1.8, label="KALMAN")[0],
        }
        ax.set_xlabel("Thoi gian [s]")
        ax.set_ylabel("roll [do]")
        ax.grid(True, alpha=0.25, ls=":")
        ax.legend(loc="upper left", fontsize=9)
        ax.set_title("MPU6050 roll - realtime tu ESP32")

    n_data = n_skip = 0
    t_start = time.time()
    t_last_draw = 0.0

    with open(args.out, "w") as f:
        f.write("#" + ",".join(COLUMNS) + "\n")
        try:
            while True:
                if args.duration and (time.time() - t_start) >= args.duration:
                    break

                raw = ser.readline()
                if not raw:
                    continue
                line = raw.decode("utf-8", errors="replace").strip()
                if not line:
                    continue

                # Firmware in tieu de bat dau bang '#'; ESP_LOG in cac dong khac.
                # Chi nhan dong phan tich duoc thanh dung so cot.
                parts = line.split(",")
                if len(parts) != len(COLUMNS):
                    if not line.startswith("#"):
                        n_skip += 1
                        # In log cua firmware ra de nguoi dung thay tien trinh
                        # (hieu chinh, loi I2C...) thay vi im lang.
                        if any(tag in line for tag in ("E (", "W (", "I (")):
                            print("  [board] " + line)
                    continue
                try:
                    vals = [float(x) for x in parts]
                except ValueError:
                    n_skip += 1
                    continue

                f.write(line + "\n")
                n_data += 1
                if n_data % 250 == 0:
                    f.flush()   # de mat dien khong mat het du lieu
                    print(f"\r  {n_data} mau ({n_skip} dong bo qua)", end="", flush=True)

                if args.live:
                    d = dict(zip(COLUMNS, vals))
                    hist["t"].append(d["t_ms"] / 1000.0)
                    hist["acc"].append(d["roll_acc"])
                    hist["comp"].append(d["roll_comp"])
                    hist["kf"].append(d["roll_kf"])

                    # Ve toi da 10 lan/giay: ve moi mau se khong theo kip 50 Hz
                    # va lam treo vong doc serial.
                    now = time.time()
                    if now - t_last_draw > 0.1:
                        t_last_draw = now
                        t = list(hist["t"])
                        for k, ln in lines.items():
                            ln.set_data(t, list(hist[k]))
                        if t:
                            ax.set_xlim(max(0, t[-1] - args.window), max(args.window, t[-1]))
                        ax.relim()
                        ax.autoscale_view(scaley=True, scalex=False)
                        fig.canvas.draw_idle()
                        fig.canvas.flush_events()

        except KeyboardInterrupt:
            print("\nDung.")
        finally:
            ser.close()

    print(f"\nDa ghi {n_data} mau vao {args.out} ({n_skip} dong khong phai du lieu).")
    if n_data == 0:
        print("Khong nhan duoc dong du lieu nao. Kiem tra:")
        print("  - Baud rate co khop CONFIG_ESP_CONSOLE_UART_BAUDRATE khong")
        print("  - Firmware da qua giai doan hieu chinh chua (mat khoang 8 giay)")
        print("  - idf.py monitor co dang chiem cong khong (chi mot tien trinh mo duoc)")
    else:
        print(f"Ve do thi: tools/.venv/bin/python tools/plot_csv.py {args.out}")


if __name__ == "__main__":
    main()
