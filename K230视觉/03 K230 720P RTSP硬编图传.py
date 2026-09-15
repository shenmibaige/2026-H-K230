# -*- coding: utf-8 -*-
# K230 CanMV v1.8 / 立创·庐山派
#
# 直播链路：1920x1080@30 全幅采集 -> 1280x720 YUV420SP
#          -> H.264 Main (VENC) -> RTSP -> VLC
#
# 不要与“03 VLC图传测试.py”同时运行。后者是低清 JPEG/HTTP-MJPEG 回退方案。
# 本程序不把 RGB565 图像逐帧转 JPEG，也不在编码循环中 sleep。

import os
import sys
import time
import network
import _thread
import uctypes

from media.vencoder import *
from media.sensor import *
from media.media import *
import multimedia as mm


# ============================== 用户配置 ==============================
PROGRAM_VERSION = "2026-07-30-r3"

# 手机热点必须设置为独立 2.4 GHz；庐山派板载 RTL8189FTV 不支持 5 GHz/双频合一。
# 为避免把真实凭据提交到工程，首次使用前请只在本机填写下列两项。
WIFI_SSID = "myphone"
WIFI_PWD = "15892738529"

# None 会调用 Sensor(...) 自动选择当前固件探测到的默认摄像头接口；
# 只有在官方 RTSP 例程已确认实际 camera id 时，才改为对应的 0/1/2。
CAMERA_ID = None
SENSOR_MODE_WIDTH = 1920
SENSOR_MODE_HEIGHT = 1080
SENSOR_MODE_FPS = 30
OUTPUT_WIDTH = 1280
OUTPUT_HEIGHT = 720
TARGET_FPS = 30                 # 板端目标值；接收端验收线为 >= 24 fps。
GOP_LENGTH = 30                 # 每秒一个 IDR，兼顾低延时和 VLC 重连。
BITRATE_KBPS = 4000             # CanMV v1.8 ChnAttrStr 的初始码率，单位 kbit/s。
VENC_BUFFER_COUNT = 4           # 低积压默认值。
VENC_BUFFER_FALLBACK_COUNT = 6  # 4 缓冲出现任一 GetStream/缓冲错误时的唯一回退值。
GET_STREAM_TIMEOUT_MS = 200   # v1.8 API 的超时单位为 ms；用于保证 stop 可以收敛。
STARTUP_GATE_MS = 3000
STARTUP_FPS_MIN = 24.0
STARTUP_FPS_MAX = 35.0
ENCODER_FPS_MIN = 29.0
ENCODER_FPS_MAX = 31.0

RTSP_PORT = 8554
RTSP_SESSION = "ball"
WIFI_TIMEOUT_S = 20
WIFI_RETRY_COUNT = 3

# 先用 encoder_only 模式完成“Sensor -> VENC” 60 秒基准测试，再改回 rtsp。
RUN_MODE_RTSP = "rtsp"
RUN_MODE_ENCODER_ONLY = "encoder_only"
RUN_MODE = RUN_MODE_RTSP
ENCODER_ONLY_SECONDS = 60

# 正常停止时等待编码线程退出的上限；超时后不强行释放仍被线程引用的媒体资源。
STOP_WAIT_MS = 4000


def _ticks_ms():
    """兼容 MicroPython/CanMV 的单调毫秒计时。"""
    if hasattr(time, "ticks_ms"):
        return time.ticks_ms()
    return int(time.time() * 1000)


def _ticks_diff(now, then):
    if hasattr(time, "ticks_diff"):
        return time.ticks_diff(now, then)
    return now - then


def _ticks_us():
    """用于统计单次 RTSP 发送耗时；无微秒计时时退回毫秒。"""
    if hasattr(time, "ticks_us"):
        return time.ticks_us()
    return _ticks_ms() * 1000


def _ticks_us_diff(now, then):
    if hasattr(time, "ticks_diff"):
        return time.ticks_diff(now, then)
    return now - then


def _sleep_ms(milliseconds):
    if hasattr(time, "sleep_ms"):
        time.sleep_ms(milliseconds)
    else:
        time.sleep(milliseconds / 1000)


def _exitpoint():
    """保留 CanMV IDE 的停止按钮响应；CPython 静态检查时不会报错。"""
    try:
        os.exitpoint()
    except AttributeError:
        pass


def _enable_exitpoint():
    try:
        os.exitpoint(os.EXITPOINT_ENABLE)
    except AttributeError:
        pass


def _print_exception(prefix, error):
    print(prefix, error)
    try:
        sys.print_exception(error)
    except AttributeError:
        pass


def _read_rssi(wlan):
    """不同 CanMV 构建返回的 status 对象略有差异，统一为可打印值。"""
    if wlan is None:
        return "?"
    try:
        status = wlan.status()
        if isinstance(status, dict):
            return status.get("rssi", "?")
        if hasattr(status, "rssi"):
            return status.rssi
    except BaseException:
        pass
    return "?"


def _wifi_ready():
    return (WIFI_SSID and WIFI_PWD and
            not WIFI_SSID.startswith("YOUR_") and
            not WIFI_PWD.startswith("YOUR_"))


def _print_wifi_config(wlan):
    config = wlan.ifconfig()
    print("Wi-Fi 网络配置: IP=%s MASK=%s GW=%s DNS=%s" % (
        config[0], config[1], config[2], config[3]
    ))


def connect_wifi():
    """连接手机热点。连接失败时不启动任何媒体管线。"""
    wlan = network.WLAN(network.STA_IF)
    if not wlan.active():
        wlan.active(True)

    if wlan.isconnected() and wlan.ifconfig()[0] != "0.0.0.0":
        print("复用当前已连接的 Wi-Fi。")
        _print_wifi_config(wlan)
        return wlan

    if not _wifi_ready():
        raise ValueError(
            "RTSP 模式需要 Wi-Fi：请填写 WIFI_SSID/WIFI_PWD，"
            "或先让 K230 连上热点。不要在 VLC 中输入字面量 <K230_IP>。"
        )

    for attempt in range(1, WIFI_RETRY_COUNT + 1):
        print("连接 Wi-Fi（第 %d/%d 次）:" % (attempt, WIFI_RETRY_COUNT), WIFI_SSID)
        try:
            wlan.connect(WIFI_SSID, WIFI_PWD)
        except BaseException as error:
            _print_exception("Wi-Fi connect 调用失败:", error)

        deadline = _ticks_ms()
        while _ticks_diff(_ticks_ms(), deadline) < WIFI_TIMEOUT_S * 1000:
            if wlan.isconnected() and wlan.ifconfig()[0] != "0.0.0.0":
                print("Wi-Fi 已连接，RSSI:", _read_rssi(wlan), "dBm")
                _print_wifi_config(wlan)
                return wlan
            _sleep_ms(200)

        print("Wi-Fi 连接超时。确认手机热点已强制设置为 2.4 GHz，且未使用双频合一。")

    return None


class H264RtspStreamer:
    """单路 Sensor -> VENC -> RTSP 推流器。

    该类严格沿用 CanMV v1.8 官方 RTSP 示例的媒体初始化顺序：
    SetOutBufs -> link -> Create -> RTSP -> Start -> sensor.run。
    v1.8 的 MediaManager.init/deinit 已标为废弃接口，故不调用。
    """

    def __init__(self, run_mode):
        self.run_mode = run_mode
        self.venc_chn = VENC_CHN_ID_0
        self.sensor = None
        self.encoder = None
        self.link = None
        self.rtspserver = None
        self.sensor_source_fps = None
        self.sensor_name = None

        self._sensor_reset = False
        self._encoder_created = False
        self._encoder_started = False
        self._sensor_started = False
        self._rtsp_initialized = False
        self._rtsp_started = False
        self._thread_started = False
        self._thread_done = True
        self._cleanup_complete = False
        self.running = False
        self.fatal_error = None

        self.total_frames = 0
        self.total_packs = 0
        self.total_encoded_bytes = 0
        self.total_rtsp_bytes = 0
        self.venc_errors = 0
        self.getstream_errors = 0
        self.release_errors = 0
        self.rtsp_errors = 0
        self.pts_zero = 0
        self.pts_regressions = 0
        self.buffer_fallback_required = False

        self._stream_start_ms = None
        self._startup_gate_checked = False
        self._last_rtsp_pts_us = 0
        self._last_rtsp_timestamp_ms = 0
        self._first_frame_pts_us = None
        self._last_frame_pts_us = None
        self._valid_pts_frames = 0
        self._pts_origin_us = None
        self._pts_wall_origin_ms = None
        self.queue_drift_ms = 0.0
        self._send_max_ms_report = 0.0

        self._last_report_ms = _ticks_ms()
        self._last_report_frames = 0
        self._last_report_encoded_bytes = 0
        self._last_report_rtsp_bytes = 0
        self._last_report_pts_us = None
        self._last_report_valid_pts_frames = 0
        self._fps_samples = []

    @staticmethod
    def _require_success(label, result):
        """CanMV C 扩展通常以 0/None 表示成功；其他返回码必须显式失败。"""
        if result is not None and result != 0:
            raise RuntimeError("%s 失败，返回码=%s" % (label, result))

    def _get_stream(self, stream_data):
        """兼容 v1.8 的显式 timeout 参数及少数旧构建的两参签名。"""
        try:
            return self.encoder.GetStream(
                self.venc_chn, stream_data, timeout=GET_STREAM_TIMEOUT_MS
            )
        except TypeError:
            try:
                return self.encoder.GetStream(
                    self.venc_chn, stream_data, GET_STREAM_TIMEOUT_MS
                )
            except TypeError:
                return self.encoder.GetStream(self.venc_chn, stream_data)

    def _require_sensor_mode_available(self):
        """在创建 Sensor 前枚举模式；没有精确 1080P30 时禁止静默回退。"""
        try:
            if CAMERA_ID is None:
                result = Sensor.list_mode()
            else:
                result = Sensor.list_mode(id=CAMERA_ID)
        except BaseException as error:
            raise RuntimeError(
                "Sensor.list_mode() 失败；本程序要求 K230 CanMV v1.8 "
                "并且必须先确认 1920x1080@30。原始错误=%s" % error
            )

        if not result or len(result) != 2:
            raise RuntimeError("Sensor.list_mode() 未返回 (sensor_name, modes)")

        sensor_name, modes = result
        self.sensor_name = sensor_name
        required_found = False
        print("MODE_CHECK_SENSOR=%s" % sensor_name)
        for mode in modes:
            try:
                mode_width = int(mode["width"])
                mode_height = int(mode["height"])
                mode_fps = float(mode["fps"])
            except BaseException:
                print("MODE_CHECK_UNPARSED=%s" % mode)
                continue

            if (mode_width == SENSOR_MODE_WIDTH and
                    mode_height == SENSOR_MODE_HEIGHT and
                    abs(mode_fps - SENSOR_MODE_FPS) < 0.01):
                required_found = True

        if not required_found:
            print("REQUIRED_MODE_MISSING=%dx%d@%d" % (
                SENSOR_MODE_WIDTH, SENSOR_MODE_HEIGHT, SENSOR_MODE_FPS
            ))
            raise RuntimeError(
                "当前摄像头模式表中没有精确的 %dx%d@%d；"
                "拒绝退回会损失视场的 1280x720@90。" % (
                    SENSOR_MODE_WIDTH, SENSOR_MODE_HEIGHT, SENSOR_MODE_FPS
                )
            )

        print("REQUIRED_MODE_FOUND=%dx%d@%d" % (
            SENSOR_MODE_WIDTH, SENSOR_MODE_HEIGHT, SENSOR_MODE_FPS
        ))

    def _read_actual_acquisition_mode(self):
        """读取 reset 后固件实际选中的采集模式，而不是构造函数请求值。"""
        try:
            actual_width = int(self.sensor.width(chn=None))
            actual_height = int(self.sensor.height(chn=None))
            actual_fps = float(self.sensor._dev_attr.sensor_info.fps)
        except BaseException as error:
            raise RuntimeError(
                "无法读取 Sensor 实际采集模式；不能证明已选中全幅 1080P30。"
                "原始错误=%s" % error
            )
        return actual_width, actual_height, actual_fps

    def _verify_actual_acquisition_mode(self, stage):
        actual_width, actual_height, actual_fps = self._read_actual_acquisition_mode()
        if (actual_width != SENSOR_MODE_WIDTH or
                actual_height != SENSOR_MODE_HEIGHT or
                abs(actual_fps - SENSOR_MODE_FPS) >= 0.01):
            raise RuntimeError(
                "%s 实际采集模式为 %dx%d@%.3f，不是要求的 %dx%d@%d；"
                "拒绝继续推流。" % (
                    stage,
                    actual_width, actual_height, actual_fps,
                    SENSOR_MODE_WIDTH, SENSOR_MODE_HEIGHT, SENSOR_MODE_FPS,
                )
            )
        self.sensor_source_fps = int(round(actual_fps))
        print("ACQ_MODE=%dx%d@%d stage=%s" % (
            actual_width, actual_height, self.sensor_source_fps, stage
        ))

    def _verify_output_geometry(self):
        try:
            actual_width = int(self.sensor.width(chn=CAM_CHN_ID_0))
            actual_height = int(self.sensor.height(chn=CAM_CHN_ID_0))
        except BaseException as error:
            raise RuntimeError("无法读取 Sensor 通道输出尺寸。原始错误=%s" % error)

        if actual_width != OUTPUT_WIDTH or actual_height != OUTPUT_HEIGHT:
            raise RuntimeError(
                "通道实际输出为 %dx%d，不是要求的 %dx%d" % (
                    actual_width, actual_height, OUTPUT_WIDTH, OUTPUT_HEIGHT
                )
            )

        try:
            crop_enabled = bool(
                self.sensor._chn_attr[CAM_CHN_ID_0].crop_enable
            )
        except BaseException as error:
            raise RuntimeError(
                "无法确认 crop_enable；不能证明输出保留完整视场。原始错误=%s" % error
            )

        if crop_enabled:
            raise RuntimeError("Sensor 通道仍启用了裁剪；拒绝继续推流")
        print("OUTPUT=%dx%d CROP=off" % (actual_width, actual_height))

    def _create_sensor(self):
        """强制 GC2093 选用全幅 1080P30，再由通道缩放为不裁剪的 720P。"""
        if CAMERA_ID is None:
            self.sensor = Sensor(
                width=SENSOR_MODE_WIDTH,
                height=SENSOR_MODE_HEIGHT,
                fps=SENSOR_MODE_FPS,
            )
        else:
            self.sensor = Sensor(
                id=CAMERA_ID,
                width=SENSOR_MODE_WIDTH,
                height=SENSOR_MODE_HEIGHT,
                fps=SENSOR_MODE_FPS,
            )

        try:
            self.sensor.reset()
            self._sensor_reset = True
            self._verify_actual_acquisition_mode("after_reset")
            self.sensor.set_framesize(
                width=OUTPUT_WIDTH,
                height=OUTPUT_HEIGHT,
                alignment=12,
                crop=None,
            )
            self.sensor.set_pixformat(Sensor.YUV420SP)
            self._verify_output_geometry()
            return self.sensor
        except BaseException:
            # 构造成功但配置失败时也释放 CSI 占用，便于用户修正后直接重跑。
            try:
                self.sensor.stop(is_del=True)
            except BaseException:
                pass
            self.sensor = None
            self._sensor_reset = False
            raise

    def _create_encoder_attr(self):
        try:
            # CanMV v1.8 的准确字段：bit_rate、gopLen、src_frame_rate、dst_frame_rate。
            attr = ChnAttrStr(
                self.encoder.PAYLOAD_TYPE_H264,
                self.encoder.H264_PROFILE_MAIN,
                OUTPUT_WIDTH,
                OUTPUT_HEIGHT,
                bit_rate=BITRATE_KBPS,
                gopLen=GOP_LENGTH,
                src_frame_rate=TARGET_FPS,
                dst_frame_rate=TARGET_FPS,
            )
            print("VENC 属性: src_fps=%d dst_fps=%d GOP=%d bitrate=%dkb/s buffers=%d" % (
                TARGET_FPS, TARGET_FPS, GOP_LENGTH, BITRATE_KBPS,
                VENC_BUFFER_COUNT,
            ))
            return attr
        except TypeError as error:
            raise RuntimeError(
                "ChnAttrStr 与 CanMV v1.8 签名不一致；"
                "拒绝使用无法确认帧率/码率的兼容回退。原始错误=%s" % error
            )

    def _setup_video_pipeline(self):
        self._require_sensor_mode_available()
        self.sensor = self._create_sensor()
        self.encoder = Encoder()

        # v1.8 官方 RTSP 例程：先分配 VENC 输出缓冲，再建立 Sensor -> VENC 绑定。
        self._require_success(
            "Encoder.SetOutBufs",
            self.encoder.SetOutBufs(
                self.venc_chn,
                VENC_BUFFER_COUNT,
                OUTPUT_WIDTH,
                OUTPUT_HEIGHT,
            ),
        )
        self.link = MediaManager.link(
            self.sensor.bind_info()["src"],
            (VIDEO_ENCODE_MOD_ID, VENC_DEV_ID, self.venc_chn),
        )

        self._require_success(
            "Encoder.Create",
            self.encoder.Create(self.venc_chn, self._create_encoder_attr()),
        )
        self._encoder_created = True

    def _setup_rtsp(self):
        self.rtspserver = mm.rtsp_server()
        self._require_success("RTSP init", self.rtspserver.rtspserver_init(RTSP_PORT))
        self._rtsp_initialized = True
        self._require_success(
            "RTSP createsession",
            self.rtspserver.rtspserver_createsession(
                RTSP_SESSION,
                mm.multi_media_type.media_h264,
                False,  # 本题只需要视频，关闭音频减少资源占用。
            ),
        )
        self._require_success("RTSP start", self.rtspserver.rtspserver_start())
        self._rtsp_started = True

    def start(self):
        if self._cleanup_complete:
            raise RuntimeError("同一对象不能在 stop 后重启；请新建 H264RtspStreamer 实例。")

        try:
            self._setup_video_pipeline()
            if self.run_mode == RUN_MODE_RTSP:
                self._setup_rtsp()

            self._require_success("Encoder.Start", self.encoder.Start(self.venc_chn))
            self._encoder_started = True
            self.sensor.run()
            self._sensor_started = True
            self._verify_actual_acquisition_mode("after_run")
            self._verify_output_geometry()

            self.running = True
            # 媒体初始化可能超过 1 秒；从真正开始取流的时刻重置统计窗口，
            # 避免首条 STAT 因初始化耗时显示为 0 fps。
            self._stream_start_ms = _ticks_ms()
            self._last_report_ms = self._stream_start_ms
            self._last_report_frames = self.total_frames
            self._last_report_encoded_bytes = self.total_encoded_bytes
            self._last_report_rtsp_bytes = self.total_rtsp_bytes
            self._last_report_pts_us = None
            self._last_report_valid_pts_frames = self._valid_pts_frames
            self._send_max_ms_report = 0.0
            self._fps_samples = []
            self._thread_done = False
            _thread.start_new_thread(self._stream_loop, ())
            self._thread_started = True
        except BaseException:
            self.running = False
            self._teardown()
            raise

    def get_rtsp_url(self, ip_address):
        if self.run_mode != RUN_MODE_RTSP:
            return None
        # 明确返回数字 IPv4，避免用户把说明文档中的 <K230_IP> 占位符粘贴进 VLC。
        return "rtsp://%s:%d/%s" % (ip_address, RTSP_PORT, RTSP_SESSION)

    def _rtsp_timestamp(self, pts_us):
        """把 VENC 微秒 PTS 转成 RTSP 毫秒时间戳，并防止零值或倒退。"""
        try:
            pts_us = int(pts_us)
        except (TypeError, ValueError):
            pts_us = 0

        if pts_us <= 0:
            self.pts_zero += 1
            return self._last_rtsp_timestamp_ms

        if self._last_rtsp_pts_us > 0 and pts_us < self._last_rtsp_pts_us:
            self.pts_regressions += 1
            return self._last_rtsp_timestamp_ms

        timestamp_ms = pts_us // 1000
        if timestamp_ms < self._last_rtsp_timestamp_ms:
            self.pts_regressions += 1
            return self._last_rtsp_timestamp_ms

        self._last_rtsp_pts_us = pts_us
        self._last_rtsp_timestamp_ms = timestamp_ms
        return timestamp_ms

    def _record_frame_pts(self, pts_us):
        """按帧记录唯一递增 PTS，并估算板端队列相对漂移。"""
        try:
            pts_us = int(pts_us)
        except (TypeError, ValueError):
            return

        if pts_us <= 0:
            return
        if self._last_frame_pts_us is not None and pts_us <= self._last_frame_pts_us:
            return

        wall_now_ms = _ticks_ms()
        if self._first_frame_pts_us is None:
            self._first_frame_pts_us = pts_us
            self._pts_origin_us = pts_us
            self._pts_wall_origin_ms = wall_now_ms
            self.queue_drift_ms = 0.0

        self._last_frame_pts_us = pts_us
        self._valid_pts_frames += 1
        wall_elapsed_ms = _ticks_diff(wall_now_ms, self._pts_wall_origin_ms)
        pts_elapsed_ms = (pts_us - self._pts_origin_us) / 1000.0
        self.queue_drift_ms = wall_elapsed_ms - pts_elapsed_ms

    @staticmethod
    def _first_positive_pts(stream_data):
        for pack_idx in range(0, stream_data.pack_cnt):
            try:
                pts_us = int(stream_data.pts[pack_idx])
            except (TypeError, ValueError):
                continue
            if pts_us > 0:
                return pts_us
        return 0

    def _check_startup_fps_gate(self, now_ms):
        """前三秒只看真实取流计数；超出范围立即停止，避免 93 fps 假通过。"""
        if self._startup_gate_checked or self._stream_start_ms is None:
            return True

        elapsed_ms = _ticks_diff(now_ms, self._stream_start_ms)
        if elapsed_ms < STARTUP_GATE_MS:
            return True

        self._startup_gate_checked = True
        startup_fps = self.total_frames * 1000.0 / elapsed_ms
        print("[STARTUP] elapsed_ms=%d frames=%d avg_fps=%.1f allowed=%.1f..%.1f" % (
            elapsed_ms,
            self.total_frames,
            startup_fps,
            STARTUP_FPS_MIN,
            STARTUP_FPS_MAX,
        ))
        if startup_fps < STARTUP_FPS_MIN or startup_fps > STARTUP_FPS_MAX:
            error = RuntimeError(
                "STARTUP_FPS_GATE_FAIL: 前 %.1f 秒平均 %.1f fps，"
                "不在 %.1f..%.1f；推流已停止。请确认实际采集为 1080P30。" % (
                    elapsed_ms / 1000.0,
                    startup_fps,
                    STARTUP_FPS_MIN,
                    STARTUP_FPS_MAX,
                )
            )
            self.fatal_error = error
            self.running = False
            print("[FATAL]", error)
            return False

        print("STARTUP_FPS_GATE=PASS")
        return True

    def _stream_loop(self):
        stream_data = StreamData()
        try:
            while self.running:
                _exitpoint()
                stream_acquired = False
                try:
                    try:
                        result = self._get_stream(stream_data)
                    except BaseException as error:
                        self.getstream_errors += 1
                        self.buffer_fallback_required = (
                            VENC_BUFFER_COUNT == 4
                        )
                        raise RuntimeError(
                            "GetStream 调用异常；当前 VENC buffers=%d。"
                            "若为 4，请改为唯一回退值 %d 后完整复测。原始错误=%s" % (
                                VENC_BUFFER_COUNT,
                                VENC_BUFFER_FALLBACK_COUNT,
                                error,
                            )
                        )

                    if result not in (None, 0):
                        self.getstream_errors += 1
                        self.buffer_fallback_required = (
                            VENC_BUFFER_COUNT == 4
                        )
                        raise RuntimeError(
                            "GetStream 失败，返回码=%s，当前 VENC buffers=%d。"
                            "若为 4，请改为唯一回退值 %d 后完整复测。" % (
                                result,
                                VENC_BUFFER_COUNT,
                                VENC_BUFFER_FALLBACK_COUNT,
                            )
                        )

                    stream_acquired = True
                    self.total_frames += 1

                    if (stream_data.pack_cnt <= 0 or
                            stream_data.pack_cnt > len(stream_data.data)):
                        raise RuntimeError("非法 VENC pack_cnt=%s" % stream_data.pack_cnt)

                    self._record_frame_pts(self._first_positive_pts(stream_data))

                    for pack_idx in range(0, stream_data.pack_cnt):
                        packet_size = stream_data.data_size[pack_idx]
                        packet_addr = stream_data.data[pack_idx]
                        if packet_size <= 0 or packet_addr == 0:
                            raise RuntimeError("非法 VENC pack: index=%d size=%s addr=%s" % (
                                pack_idx, packet_size, packet_addr
                            ))
                        self.total_packs += 1
                        self.total_encoded_bytes += packet_size

                        if self.run_mode == RUN_MODE_RTSP:
                            try:
                                packet = bytes(uctypes.bytearray_at(
                                    packet_addr, packet_size
                                ))
                                send_begin_us = _ticks_us()
                                try:
                                    send_result = self.rtspserver.rtspserver_sendvideodata(
                                        RTSP_SESSION,
                                        packet,
                                        packet_size,
                                        self._rtsp_timestamp(
                                            stream_data.pts[pack_idx]
                                        ),
                                    )
                                finally:
                                    send_elapsed_ms = _ticks_us_diff(
                                        _ticks_us(), send_begin_us
                                    ) / 1000.0
                                    if send_elapsed_ms > self._send_max_ms_report:
                                        self._send_max_ms_report = send_elapsed_ms
                                self._require_success("RTSP sendvideodata", send_result)
                                self.total_rtsp_bytes += packet_size
                            except BaseException as error:
                                self.rtsp_errors += 1
                                # 连接端暂时离开时不能让日志和异常风暴拖垮编码线程。
                                if self.rtsp_errors == 1 or self.rtsp_errors % 30 == 0:
                                    _print_exception("RTSP 发送异常:", error)
                except BaseException as error:
                    self.venc_errors += 1
                    self.fatal_error = error
                    _print_exception("VENC GetStream/处理异常:", error)
                    self.running = False
                finally:
                    if stream_acquired:
                        try:
                            self._require_success(
                                "Encoder.ReleaseStream",
                                self.encoder.ReleaseStream(self.venc_chn, stream_data),
                            )
                        except BaseException as error:
                            self.venc_errors += 1
                            self.release_errors += 1
                            self.fatal_error = error
                            _print_exception("VENC ReleaseStream 异常:", error)
                            self.running = False
        finally:
            self._thread_done = True

    def report(self, wlan):
        now = _ticks_ms()
        elapsed_ms = _ticks_diff(now, self._last_report_ms)
        if elapsed_ms < 1000:
            return

        frame_delta = self.total_frames - self._last_report_frames
        encoded_delta = self.total_encoded_bytes - self._last_report_encoded_bytes
        rtsp_delta = self.total_rtsp_bytes - self._last_report_rtsp_bytes
        fps = frame_delta * 1000.0 / elapsed_ms
        encoded_kbps = encoded_delta * 8.0 / elapsed_ms
        rtsp_kbps = rtsp_delta * 8.0 / elapsed_ms

        current_pts_us = self._last_frame_pts_us
        current_pts_frames = self._valid_pts_frames
        pts_fps = 0.0
        if current_pts_us is not None:
            if (self._last_report_pts_us is not None and
                    current_pts_us > self._last_report_pts_us):
                pts_frame_delta = (
                    current_pts_frames - self._last_report_valid_pts_frames
                )
                pts_elapsed_us = current_pts_us - self._last_report_pts_us
            elif (self._first_frame_pts_us is not None and
                    current_pts_us > self._first_frame_pts_us):
                pts_frame_delta = current_pts_frames - 1
                pts_elapsed_us = current_pts_us - self._first_frame_pts_us
            else:
                pts_frame_delta = 0
                pts_elapsed_us = 0

            if pts_frame_delta > 0 and pts_elapsed_us > 0:
                pts_fps = pts_frame_delta * 1000000.0 / pts_elapsed_us

        pts_step_ms = 1000.0 / pts_fps if pts_fps > 0 else 0.0
        send_max_ms = self._send_max_ms_report
        self._fps_samples.append(fps)
        print("[STAT] enc_fps=%.1f pts_fps=%.1f pts_step_ms=%.1f "
              "queue_drift_ms=%.1f send_max_ms=%.1f packs=%d "
              "enc=%.0fkb/s rtsp=%.0fkb/s venc_err=%d get_err=%d "
              "rel_err=%d rtsp_err=%d pts_zero=%d pts_back=%d rssi=%sdBm" % (
            fps,
            pts_fps,
            pts_step_ms,
            self.queue_drift_ms,
            send_max_ms,
            self.total_packs,
            encoded_kbps,
            rtsp_kbps,
            self.venc_errors,
            self.getstream_errors,
            self.release_errors,
            self.rtsp_errors,
            self.pts_zero,
            self.pts_regressions,
            _read_rssi(wlan),
        ))

        self._last_report_ms = now
        self._last_report_frames = self.total_frames
        self._last_report_encoded_bytes = self.total_encoded_bytes
        self._last_report_rtsp_bytes = self.total_rtsp_bytes
        self._last_report_pts_us = current_pts_us
        self._last_report_valid_pts_frames = current_pts_frames
        self._send_max_ms_report = 0.0
        self._check_startup_fps_gate(now)

    def print_encoder_only_result(self):
        """输出机器可读的 60 秒编码基准结论；高于目标太多同样判为失败。"""
        samples = self._fps_samples
        if len(samples) > 2:
            samples = samples[1:]

        if not samples:
            print("[RESULT] FAIL: 没有有效的编码帧率样本")
            return False

        average_fps = sum(samples) / len(samples)
        min_fps = min(samples)
        max_fps = max(samples)
        pts_fps = 0.0
        if (self._valid_pts_frames > 1 and
                self._first_frame_pts_us is not None and
                self._last_frame_pts_us > self._first_frame_pts_us):
            pts_fps = (
                (self._valid_pts_frames - 1) * 1000000.0 /
                (self._last_frame_pts_us - self._first_frame_pts_us)
            )

        errors = self.venc_errors + self.rtsp_errors
        passed = (
            average_fps >= ENCODER_FPS_MIN and
            average_fps <= ENCODER_FPS_MAX and
            pts_fps >= ENCODER_FPS_MIN and
            pts_fps <= ENCODER_FPS_MAX and
            errors == 0
        )
        print("[RESULT] %s: avg_fps=%.1f min_fps=%.1f max_fps=%.1f "
              "pts_fps=%.1f target=%d acq_fps=%d errors=%d" % (
                  "PASS" if passed else "FAIL",
                  average_fps, min_fps, max_fps,
                  pts_fps,
                  TARGET_FPS, self.sensor_source_fps, errors,
        ))
        if average_fps > ENCODER_FPS_MAX:
            print("失败原因: 实际 GetStream 帧率高于 %.1f；"
                  "采集模式或固件行为与 1080P30 断言不一致。" % ENCODER_FPS_MAX)
        elif average_fps < ENCODER_FPS_MIN:
            print("失败原因: 编码平均帧率低于 %.1f fps。" % ENCODER_FPS_MIN)
        elif pts_fps < ENCODER_FPS_MIN or pts_fps > ENCODER_FPS_MAX:
            print("失败原因: VENC PTS 推导帧率 %.1f fps 不在 %.1f..%.1f。" % (
                pts_fps, ENCODER_FPS_MIN, ENCODER_FPS_MAX
            ))
        return passed

    def _teardown(self):
        if self._cleanup_complete:
            return

        # 线程已经结束后再撤销 VENC/RTSP，避免 ReleaseStream 与 deinit 发生竞争。
        # 顺序遵循 v1.8 官方示例：sensor -> link -> VENC -> RTSP。
        if self._sensor_reset and self.sensor:
            try:
                self.sensor.stop()
            except BaseException as error:
                _print_exception("Sensor stop 异常:", error)
            self._sensor_started = False
            self._sensor_reset = False

        if self.link is not None:
            try:
                del self.link
            except BaseException as error:
                _print_exception("Media link 释放异常:", error)
            self.link = None

        if self._encoder_started and self.encoder:
            try:
                self.encoder.Stop(self.venc_chn)
            except BaseException as error:
                _print_exception("VENC stop 异常:", error)
            self._encoder_started = False

        if self._encoder_created and self.encoder:
            try:
                self.encoder.Destroy(self.venc_chn)
            except BaseException as error:
                _print_exception("VENC destroy 异常:", error)
            self._encoder_created = False

        if self._rtsp_started:
            try:
                self.rtspserver.rtspserver_stop()
            except BaseException as error:
                _print_exception("RTSP stop 异常:", error)
            self._rtsp_started = False

        if self._rtsp_initialized:
            try:
                self.rtspserver.rtspserver_deinit()
            except BaseException as error:
                _print_exception("RTSP deinit 异常:", error)
            self._rtsp_initialized = False

        self._cleanup_complete = True

    def stop(self):
        """先停止线程并等待 ReleaseStream，再按反向顺序释放硬件资源。"""
        self.running = False
        if self._thread_started and not self._thread_done:
            stop_begin = _ticks_ms()
            while not self._thread_done:
                if _ticks_diff(_ticks_ms(), stop_begin) >= STOP_WAIT_MS:
                    print("编码线程未在 %dms 内退出；不强制 deinit，重启程序/开发板后再试。" % STOP_WAIT_MS)
                    return False
                _sleep_ms(20)

        self._teardown()
        return True


def _run_service_loop(streamer, wlan):
    started_ms = _ticks_ms()
    while streamer.running:
        _exitpoint()
        streamer.report(wlan)
        if not streamer.running:
            break

        if wlan is not None and not wlan.isconnected():
            print("Wi-Fi 已断开。为避免 RTSP/VENC 资源状态不一致，程序将安全退出；请恢复热点后重新启动。")
            break

        if streamer.fatal_error is not None:
            break

        if (streamer.run_mode == RUN_MODE_ENCODER_ONLY and
                _ticks_diff(_ticks_ms(), started_ms) >= ENCODER_ONLY_SECONDS * 1000):
            print("encoder_only %d 秒基准测试结束。" % ENCODER_ONLY_SECONDS)
            streamer.print_encoder_only_result()
            break

        _sleep_ms(50)

    if streamer.fatal_error is not None:
        print("编码线程已终止:", streamer.fatal_error)
        if streamer.buffer_fallback_required:
            print("BUFFER_FALLBACK_REQUIRED=%d" % VENC_BUFFER_FALLBACK_COUNT)


def main():
    if RUN_MODE not in (RUN_MODE_RTSP, RUN_MODE_ENCODER_ONLY):
        raise ValueError("RUN_MODE 只能是 '%s' 或 '%s'" % (
            RUN_MODE_RTSP, RUN_MODE_ENCODER_ONLY
        ))

    _enable_exitpoint()
    print("K230 720P H.264 RTSP 图传启动")
    print("程序版本:", PROGRAM_VERSION)
    print("配置: ACQ=%dx%d@%d OUTPUT=%dx%d@%d H.264 Main GOP=%d mode=%s" % (
        SENSOR_MODE_WIDTH, SENSOR_MODE_HEIGHT, SENSOR_MODE_FPS,
        OUTPUT_WIDTH, OUTPUT_HEIGHT, TARGET_FPS,
        GOP_LENGTH, RUN_MODE
    ))
    print("VENC_BUFFERS=%d FALLBACK=%d" % (
        VENC_BUFFER_COUNT, VENC_BUFFER_FALLBACK_COUNT
    ))

    wlan = None
    if RUN_MODE == RUN_MODE_RTSP:
        try:
            wlan = connect_wifi()
        except BaseException as error:
            _print_exception("Wi-Fi 初始化失败，RTSP_NOT_READY:", error)
            print("请修正 Wi-Fi 后重新运行；未出现 RTSP_READY= 时，VLC 必然无法连接。")
            return
        if wlan is None:
            print("Wi-Fi 未连接，RTSP_NOT_READY；未启动媒体管线。")
            return
    else:
        print("RTSP_DISABLED: encoder_only 只测编码，不监听 8554，VLC 无法连接。")

    streamer = H264RtspStreamer(RUN_MODE)
    try:
        streamer.start()
        if RUN_MODE == RUN_MODE_RTSP:
            rtsp_url = streamer.get_rtsp_url(wlan.ifconfig()[0])
            print("RTSP_READY=" + rtsp_url)
            print("只把 RTSP_READY= 后面的完整地址粘贴到 VLC；不要输入 <K230_IP> 占位符。")
            print("Windows：使用“04 VLC低延迟播放.ps1”打开该地址；"
                  "先确认稳定画面，再通过“视图 -> 高级控制 -> 录制”。")
        else:
            print("仅编码基准测试：不启动 RTSP/VLC，持续 %d 秒。" % ENCODER_ONLY_SECONDS)

        _run_service_loop(streamer, wlan)
    except KeyboardInterrupt:
        print("用户中断。")
    except BaseException as error:
        _print_exception("主程序异常:", error)
    finally:
        if not streamer.stop():
            print("媒体资源未被强制释放；请重启 K230 后再运行。")
        else:
            print("图传已停止。")


if __name__ == "__main__":
    main()
