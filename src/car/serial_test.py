import serial
import serial.tools.list_ports
import threading
import time

# 常见波特率
BAUDRATES = [460800, 115200, 9600, 57600, 38400]

# 保存已找到的有效串口
valid_ports = []


def read_serial(port_name, baudrate):
    """
    尝试读取串口数据
    """
    try:
        ser = serial.Serial(
            port=port_name,
            baudrate=baudrate,
            timeout=1
        )

        print(f"[INFO] 正在检测 {port_name} @ {baudrate}")

        start_time = time.time()

        while time.time() - start_time < 3:
            if ser.in_waiting:
                data = ser.readline()

                try:
                    text = data.decode("utf-8", errors="ignore").strip()
                except:
                    text = str(data)

                if text:
                    print(f"\n[FOUND] 有效串口:")
                    print(f"串口: {port_name}")
                    print(f"波特率: {baudrate}")
                    print(f"数据: {text}\n")

                    valid_ports.append((port_name, baudrate))

                    # 持续监听
                    while True:
                        if ser.in_waiting:
                            msg = ser.readline().decode(
                                "utf-8",
                                errors="ignore"
                            ).strip()

                            if msg:
                                print(f"[{port_name}] {msg}")

                break

        ser.close()

    except Exception as e:
        pass


def scan_ports():
    """
    扫描所有串口
    """
    ports = serial.tools.list_ports.comports()

    if not ports:
        print("未发现串口")
        return

    print("发现串口：")

    for p in ports:
        print(f" - {p.device}")

    threads = []

    for port in ports:
        for baud in BAUDRATES:
            t = threading.Thread(
                target=read_serial,
                args=(port.device, baud)
            )

            t.daemon = True
            t.start()

            threads.append(t)

    for t in threads:
        t.join()


if __name__ == "__main__":
    scan_ports()
