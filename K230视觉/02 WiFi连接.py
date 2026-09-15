# 测试WiFi连接，看看能不能拿到IP
import time
import network

try:
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)

    # ⚠️ 改成你自己的WiFi名和密码，必须是2.4GHz！
    wlan.connect('myphone', '15892738529')

    print("正在连接WiFi...")
    timeout = 10
    while not wlan.isconnected() and timeout > 0:
        time.sleep_ms(1000)
        timeout -= 1
        print(".", end="")

    if wlan.isconnected():
        ip = wlan.ifconfig()[0]
        print(f"\n连接成功！K230的IP是: {ip}")
        print("记住这个IP，VLC里要用")
    else:
        print("\n连接失败，检查WiFi名密码，确认是2.4GHz")

except BaseException as e:
    print(f"异常: {e}")
#    连接成功！K230的IP是: 10.32.28.137
#    记住这个IP，VLC里要用
