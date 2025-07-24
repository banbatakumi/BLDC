import tkinter as tk
from tkinter import ttk
import serial
import serial.tools.list_ports


class MotorControllerApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Motor UART Controller")

        self.serial_port = None
        self.current_mode = 0xFF

        self.create_widgets()

    def create_widgets(self):
        # --- ポート選択 ---
        port_frame = ttk.Frame(self.root)
        port_frame.pack(pady=5)

        ttk.Label(port_frame, text="Serial Port:").pack(side=tk.LEFT)
        self.port_combo = ttk.Combobox(
            port_frame, values=self.get_serial_ports())
        self.port_combo.pack(side=tk.LEFT)
        ttk.Button(port_frame, text="Connect",
                   command=self.connect_serial).pack(side=tk.LEFT)

        # --- モードボタン ---
        mode_frame = ttk.Frame(self.root)
        mode_frame.pack(pady=10)

        ttk.Button(mode_frame, text="速度制御 (0xFF)", command=lambda: self.set_mode(
            0xFF)).pack(side=tk.LEFT, padx=5)
        ttk.Button(mode_frame, text="位置制御 (0xFE)", command=lambda: self.set_mode(
            0xFE)).pack(side=tk.LEFT, padx=5)
        ttk.Button(mode_frame, text="トルク制御 (0xFD)", command=lambda: self.set_mode(
            0xFD)).pack(side=tk.LEFT, padx=5)

        # --- スライダー ---
        self.value_label = ttk.Label(self.root, text="現在値: 0")
        self.value_label.pack(pady=5)

        self.slider = tk.Scale(self.root, from_=-150, to=150, orient=tk.HORIZONTAL,
                               length=400, resolution=1, command=self.on_slider_change)
        self.slider.pack()

    def get_serial_ports(self):
        ports = serial.tools.list_ports.comports()
        return [port.device for port in ports]

    def connect_serial(self):
        port = self.port_combo.get()
        try:
            self.serial_port = serial.Serial(port, 115200)
            print(f"接続成功: {port}")
        except Exception as e:
            print(f"接続失敗: {e}")

    def set_mode(self, mode_byte):
        self.current_mode = mode_byte
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.write(bytes([mode_byte]))
            print(f"モード切替: {hex(mode_byte)}")

        # スライダーの範囲を変更
        if mode_byte == 0xFF or mode_byte == 0xFD:  # 速度 or トルク制御
            self.slider.config(from_=-150, to=150, resolution=1)
        elif mode_byte == 0xFE:  # 位置制御 (0 ~ 2π)
            self.slider.config(from_=0.0, to=6.283, resolution=0.01)

        self.slider.set(0)
        self.value_label.config(text="現在値: 0")

    def on_slider_change(self, val):
        if not self.serial_port or not self.serial_port.is_open:
            return

        try:
            if self.current_mode == 0xFF or self.current_mode == 0xFD:
                value = int(float(val))
                byte_value = (value + 256) % 256  # 符号付き変換
                self.serial_port.write(bytes([self.current_mode, byte_value]))
                self.value_label.config(text=f"現在値: {value}")
                print(f"送信: [{hex(self.current_mode)}, {value}]")

            elif self.current_mode == 0xFE:
                position = float(val)  # ラジアン
                # 1000倍して整数にエンコード（例：3.141 → 3141）
                scaled = int(position * 1000)
                high = (scaled >> 8) & 0xFF
                low = scaled & 0xFF
                self.serial_port.write(bytes([self.current_mode, high, low]))
                self.value_label.config(text=f"現在値: {position:.2f} rad")
                print(f"送信: [{hex(self.current_mode)}, {high}, {low}]")

        except Exception as e:
            print(f"送信エラー: {e}")


if __name__ == "__main__":
    root = tk.Tk()
    app = MotorControllerApp(root)
    root.mainloop()
