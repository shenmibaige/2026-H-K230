# WiFi 图传 v1.6 - 优化版（低带宽适应）
# 功能：/stream (浏览器)、/vlc (VLC)、/snap (快照)
# K230 CanMV

import time
import socket
import network
import os
import sys
from media.sensor import *
from media.media import *

# ====================== 配置区（可自由调整）========================
WIFI_SSID   = 'myphone'          # WiFi 名称
WIFI_PWD    = '15892738529'      # WiFi 密码
HTTP_PORT   = 8080               # 服务端口

# 图像参数（降低分辨率/质量可大幅减少数据量）
RESOLUTION = Sensor.QVGA         # QVGA = 320x240（推荐） 或 Sensor.VGA
JPEG_QUALITY = 50                # 1~100，数值越小文件越小（50 平衡）
STREAM_FPS  = 10                 # 浏览器推流帧率（建议 5~10）
VLC_FPS     = 8                  # VLC 推流帧率（建议 5~8）
# ==================================================================

# ======================== WiFi 连接 ========================
def wifi_connect(ssid, pwd, timeout_s=15):
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    wlan.connect(ssid, pwd)
    print(f"正在连接 WiFi: {ssid}", end="")
    t = timeout_s
    while not wlan.isconnected() and t > 0:
        time.sleep_ms(1000)
        print(".", end="")
        t -= 1
    print()
    if not wlan.isconnected():
        return None
    ip = wlan.ifconfig()[0]
    wait = 10
    while ip.startswith("0.") and wait > 0:
        time.sleep_ms(500)
        ip = wlan.ifconfig()[0]
        wait -= 1
    if ip.startswith("0."):
        return None
    return ip

# ======================== 摄像头 ========================
sensor = None

def camera_init():
    global sensor
    sensor = Sensor()
    sensor.reset()
    sensor.set_framesize(RESOLUTION)      # 使用配置的分辨率
    sensor.set_pixformat(Sensor.RGB565)

    MediaManager.init()
    sensor.run()

    time.sleep_ms(500)
    print("跳过初始帧...")
    for _ in range(5):
        sensor.snapshot(chn=CAM_CHN_ID_0)
        time.sleep_ms(100)
    print("摄像头就绪")

def capture_jpeg():
    """捕获 JPEG，应用配置的质量参数"""
    img = sensor.snapshot(chn=CAM_CHN_ID_0)
    return img.to_jpeg(quality=JPEG_QUALITY)   # 关键优化点

# ======================== HTTP 响应 ========================
def handle_snap(conn):
    jpeg = capture_jpeg()
    conn.send(b"HTTP/1.1 200 OK\r\n")
    conn.send(b"Content-Type: image/jpeg\r\n")
    conn.send(f"Content-Length: {len(jpeg)}\r\n".encode())
    conn.send(b"Cache-Control: no-cache\r\n")
    conn.send(b"\r\n")
    conn.sendall(jpeg)
    conn.close()
    print(f"  snap: {len(jpeg)}B")

def send_mjpeg_headers(conn, boundary, vlc_mode=False):
    conn.send(b"HTTP/1.1 200 OK\r\n")
    conn.send(b"Server: K230\r\n")
    conn.send(b"Cache-Control: no-cache, no-store\r\n")
    conn.send(b"Pragma: no-cache\r\n")
    conn.send(b"Access-Control-Allow-Origin: *\r\n")
    conn.send(b"Content-Type: multipart/x-mixed-replace; boundary=" + boundary + b"\r\n")
    if vlc_mode:
        conn.send(b"Connection: close\r\n")
    else:
        conn.send(b"Connection: keep-alive\r\n")
    conn.send(b"\r\n")

def handle_stream(conn, fps=STREAM_FPS):
    boundary = b"FRAMEBOUNDARY"
    delay_ms = int(1000 / fps)

    send_mjpeg_headers(conn, boundary, vlc_mode=False)
    print(f"  浏览器推流开始 (fps={fps})")

    cnt = 0
    while True:
        os.exitpoint()
        try:
            jpeg = capture_jpeg()
            cnt += 1
            if cnt % 30 == 1:
                print(f"    帧 {cnt}, {len(jpeg)}B")

            conn.sendall(b"--" + boundary + b"\r\n")
            conn.sendall(b"Content-Type: image/jpeg\r\n")
            conn.sendall(b"\r\n")
            conn.sendall(jpeg)
            conn.sendall(b"\r\n")

            time.sleep_ms(delay_ms)

        except OSError as e:
            err = e.args[0]
            if err == 11:
                time.sleep_ms(5)
                continue
            if err in (104, 32, 54):
                break
            print(f"  ERR: {e}")
            break
        except Exception as e:
            print(f"  ERR: {e}")
            break

    try:
        conn.sendall(b"--" + boundary + b"--\r\n")
    except:
        pass
    print(f"  推流结束, 共 {cnt} 帧")

def handle_vlc(conn, fps=VLC_FPS):
    boundary = b"BOUNDARY"
    delay_ms = int(1000 / fps)

    send_mjpeg_headers(conn, boundary, vlc_mode=True)
    print(f"  VLC 推流开始 (fps={fps})")

    cnt = 0
    while True:
        os.exitpoint()
        try:
            jpeg = capture_jpeg()
            sz = len(jpeg)
            cnt += 1
            if cnt % 30 == 1:
                print(f"    帧 {cnt}, {sz}B")

            conn.sendall(b"--" + boundary + b"\r\n")
            conn.sendall(b"Content-Type: image/jpeg\r\n")
            conn.sendall(f"Content-Length: {sz}\r\n".encode())
            conn.sendall(b"\r\n")
            conn.sendall(jpeg)
            conn.sendall(b"\r\n")

            time.sleep_ms(delay_ms)

        except OSError as e:
            err = e.args[0]
            if err == 11:
                time.sleep_ms(5)
                continue
            if err in (104, 32, 54):
                break
            print(f"  ERR: {e}")
            break
        except Exception as e:
            print(f"  ERR: {e}")
            break

    try:
        conn.sendall(b"--" + boundary + b"--\r\n")
    except:
        pass
    print(f"  VLC 推流结束, 共 {cnt} 帧")

# ======================== 主程序 ========================
def main():
    global sensor, server

    ip = wifi_connect(WIFI_SSID, WIFI_PWD, timeout_s=15)
    if ip is None:
        print("WiFi 连接失败，请检查 SSID/密码/信号")
        return
    print(f"WiFi 连接成功，IP: {ip}")
    print(f"  浏览器: http://{ip}:{HTTP_PORT}/stream")
    print(f"  VLC   : http://{ip}:{HTTP_PORT}/vlc")
    print(f"  快照  : http://{ip}:{HTTP_PORT}/snap")

    print("初始化摄像头...")
    camera_init()

    test_jpeg = capture_jpeg()
    print(f"JPEG 测试: {len(test_jpeg)} bytes")
    if len(test_jpeg) < 100:
        print("JPEG 编码异常，请检查摄像头")
        return

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((ip, HTTP_PORT))
    server.listen(1)
    print(f"HTTP 服务器已启动，监听端口 {HTTP_PORT}")

    os.exitpoint(os.EXITPOINT_ENABLE)

    while True:
        os.exitpoint()
        conn = None
        try:
            conn, addr = server.accept()
        except OSError as e:
            if e.args[0] == 11:
                time.sleep_ms(50)
                continue
            raise

        print(f"客户端: {addr}")

        req = b""
        try:
            while b"\r\n" not in req:
                c = conn.recv(1)
                if not c:
                    break
                req += c
            while b"\r\n\r\n" not in req:
                c = conn.recv(1)
                if not c:
                    break
                req += c
        except:
            conn.close()
            continue

        if not req:
            conn.close()
            continue

        first_line = req.split(b"\r\n")[0] if b"\r\n" in req else b""
        if b"GET" not in first_line:
            conn.close()
            continue

        parts = first_line.split(b" ")
        path = parts[1] if len(parts) >= 2 else b"/"

        try:
            if path == b"/snap":
                handle_snap(conn)
            elif path == b"/vlc":
                handle_vlc(conn)
            else:
                handle_stream(conn)
        except Exception as e:
            print(f"  处理请求异常: {e}")
            try:
                conn.close()
            except:
                pass

# ======================== 启动 ========================
if __name__ == "__main__":
    server = None
    try:
        main()
    except KeyboardInterrupt:
        print("用户中断")
    except BaseException as e:
        print("捕获到异常:", e)
    finally:
        if server:
            server.close()
        if sensor:
            sensor.stop()
        try:
            MediaManager.deinit()
        except:
            pass
        print("已退出")
