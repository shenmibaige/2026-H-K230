import os
import ujson
import aicube
from media.sensor import *
from media.display import *
from media.media import *
from time import *
import nncase_runtime as nn
import ulab.numpy as np
import time
import utime
import image
import random
import gc
import utime
from machine import FPIOA, UART

# ==================== BEGIN 03 RTSP INTEGRATION ====================
# 07 原代码保留为主流程；这里只增加 03 的网络、VENC 与 RTSP 能力。
# 三路 Sensor 分工：CH0=LCD，CH1=720P H.264，CH2=AI。
import sys
import network
import _thread
import uctypes
from media.vencoder import *
import multimedia as mm

RTSP_PROGRAM_VERSION = "2026-08-01-07+03-r2"
RTSP_ENABLED = True
# False：图传初始化失败时仍继续运行 07 的识别与串口功能。
# True：图传失败即退出，适合必须同时验收图传的场景。
RTSP_REQUIRED = False

# 沿用 03 的热点配置；板载 RTL8189FTV 仅使用独立 2.4 GHz 热点。
WIFI_SSID = "myphone"
WIFI_PWD = "15892738529"
WIFI_TIMEOUT_S = 20
WIFI_RETRY_COUNT = 3

RTSP_SENSOR_CHN = CAM_CHN_ID_1
RTSP_VENC_CHN = VENC_CHN_ID_0
RTSP_SENSOR_MODE_WIDTH = 1920
RTSP_SENSOR_MODE_HEIGHT = 1080
RTSP_SENSOR_MODE_FPS = 30
RTSP_OUTPUT_WIDTH = 1280
RTSP_OUTPUT_HEIGHT = 720
RTSP_TARGET_FPS = 30
RTSP_GOP_LENGTH = 30
RTSP_BITRATE_KBPS = 4000
RTSP_VENC_BUFFER_COUNT = 4
RTSP_VENC_BUFFER_FALLBACK_COUNT = 6
RTSP_GET_STREAM_TIMEOUT_MS = 200
RTSP_STARTUP_GATE_MS = 3000
RTSP_STARTUP_FPS_MIN = 24.0
RTSP_STARTUP_FPS_MAX = 35.0
RTSP_PORT = 8554
RTSP_SESSION = "ball"
RTSP_STOP_WAIT_MS = 4000


def _rtsp_ticks_ms():
    if hasattr(time, "ticks_ms"):
        return time.ticks_ms()
    return int(time.time() * 1000)


def _rtsp_ticks_diff(now, then):
    if hasattr(time, "ticks_diff"):
        return time.ticks_diff(now, then)
    return now - then


def _rtsp_ticks_us():
    if hasattr(time, "ticks_us"):
        return time.ticks_us()
    return _rtsp_ticks_ms() * 1000


def _rtsp_ticks_us_diff(now, then):
    if hasattr(time, "ticks_diff"):
        return time.ticks_diff(now, then)
    return now - then


def _rtsp_sleep_ms(milliseconds):
    if hasattr(time, "sleep_ms"):
        time.sleep_ms(milliseconds)
    else:
        time.sleep(milliseconds / 1000)


def _rtsp_exitpoint():
    try:
        os.exitpoint()
    except AttributeError:
        pass


def _rtsp_print_exception(prefix, error):
    print(prefix, error)
    try:
        sys.print_exception(error)
    except AttributeError:
        pass


def _rtsp_read_rssi(wlan):
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


def _rtsp_wifi_config_ready():
    return (WIFI_SSID and WIFI_PWD and
            not WIFI_SSID.startswith("YOUR_") and
            not WIFI_PWD.startswith("YOUR_"))


def _rtsp_print_wifi_config(wlan):
    config = wlan.ifconfig()
    print("Wi-Fi 网络配置: IP=%s MASK=%s GW=%s DNS=%s" % (
        config[0], config[1], config[2], config[3]
    ))


def connect_wifi_for_rtsp():
    """连接 03 使用的手机热点；失败由 RTSP_REQUIRED 决定是否终止 07。"""
    wlan = network.WLAN(network.STA_IF)
    if not wlan.active():
        wlan.active(True)

    if wlan.isconnected() and wlan.ifconfig()[0] != "0.0.0.0":
        print("复用当前已连接的 Wi-Fi。")
        _rtsp_print_wifi_config(wlan)
        return wlan

    if not _rtsp_wifi_config_ready():
        raise ValueError("请先填写 WIFI_SSID/WIFI_PWD；未连接网络时不会启动 RTSP。")

    for attempt in range(1, WIFI_RETRY_COUNT + 1):
        print("连接 Wi-Fi（第 %d/%d 次）:" % (attempt, WIFI_RETRY_COUNT), WIFI_SSID)
        try:
            wlan.connect(WIFI_SSID, WIFI_PWD)
        except BaseException as error:
            _rtsp_print_exception("Wi-Fi connect 调用失败:", error)

        wait_begin = _rtsp_ticks_ms()
        while _rtsp_ticks_diff(_rtsp_ticks_ms(), wait_begin) < WIFI_TIMEOUT_S * 1000:
            if wlan.isconnected() and wlan.ifconfig()[0] != "0.0.0.0":
                print("Wi-Fi 已连接，RSSI:", _rtsp_read_rssi(wlan), "dBm")
                _rtsp_print_wifi_config(wlan)
                return wlan
            _rtsp_sleep_ms(200)

        print("Wi-Fi 连接超时。请确认手机热点为独立 2.4 GHz，且未使用双频合一。")

    return None


def _rtsp_require_sensor_mode_available():
    """沿用 03 的门禁：没有精确 1920x1080@30 时不静默回退。"""
    try:
        result = Sensor.list_mode()
    except BaseException as error:
        raise RuntimeError("Sensor.list_mode() 失败，无法确认 1920x1080@30：%s" % error)

    if not result or len(result) != 2:
        raise RuntimeError("Sensor.list_mode() 未返回 (sensor_name, modes)")

    sensor_name, modes = result
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

        if (mode_width == RTSP_SENSOR_MODE_WIDTH and
                mode_height == RTSP_SENSOR_MODE_HEIGHT and
                abs(mode_fps - RTSP_SENSOR_MODE_FPS) < 0.01):
            required_found = True

    if not required_found:
        raise RuntimeError(
            "当前摄像头模式表没有精确的 %dx%d@%d；拒绝让 RTSP 静默回退。" % (
                RTSP_SENSOR_MODE_WIDTH,
                RTSP_SENSOR_MODE_HEIGHT,
                RTSP_SENSOR_MODE_FPS,
            )
        )

    print("REQUIRED_MODE_FOUND=%dx%d@%d" % (
        RTSP_SENSOR_MODE_WIDTH,
        RTSP_SENSOR_MODE_HEIGHT,
        RTSP_SENSOR_MODE_FPS,
    ))


def _rtsp_verify_actual_acquisition_mode(sensor, stage):
    try:
        actual_width = int(sensor.width(chn=None))
        actual_height = int(sensor.height(chn=None))
        actual_fps = float(sensor._dev_attr.sensor_info.fps)
    except BaseException as error:
        raise RuntimeError("无法读取 Sensor 实际采集模式：%s" % error)

    if (actual_width != RTSP_SENSOR_MODE_WIDTH or
            actual_height != RTSP_SENSOR_MODE_HEIGHT or
            abs(actual_fps - RTSP_SENSOR_MODE_FPS) >= 0.01):
        raise RuntimeError(
            "%s 实际采集模式为 %dx%d@%.3f，不是要求的 %dx%d@%d；拒绝启动 RTSP。" % (
                stage,
                actual_width,
                actual_height,
                actual_fps,
                RTSP_SENSOR_MODE_WIDTH,
                RTSP_SENSOR_MODE_HEIGHT,
                RTSP_SENSOR_MODE_FPS,
            )
        )

    print("ACQ_MODE=%dx%d@%d stage=%s" % (
        actual_width, actual_height, int(round(actual_fps)), stage
    ))


class SharedSensorH264RtspStreamer:
    """复用 07 Sensor 的 CH1，将 03 的 H.264/RTSP 管线并行接入。"""

    def __init__(self, sensor):
        self.sensor = sensor
        self.venc_chn = RTSP_VENC_CHN
        self.encoder = None
        self.link = None
        self.rtspserver = None

        self._encoder_created = False
        self._encoder_started = False
        self._rtsp_initialized = False
        self._rtsp_started = False
        self._thread_started = False
        self._thread_done = True
        self._cleanup_complete = False
        self.running = False
        self.fatal_error = None
        self.buffer_fallback_required = False

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

        self._last_report_ms = _rtsp_ticks_ms()
        self._last_report_frames = 0
        self._last_report_encoded_bytes = 0
        self._last_report_rtsp_bytes = 0
        self._last_report_pts_us = None
        self._last_report_valid_pts_frames = 0
        self._fatal_reported = False

    @staticmethod
    def _require_success(label, result):
        if result is not None and result != 0:
            raise RuntimeError("%s 失败，返回码=%s" % (label, result))

    def configure_sensor_channel(self):
        self.sensor.set_framesize(
            width=RTSP_OUTPUT_WIDTH,
            height=RTSP_OUTPUT_HEIGHT,
            chn=RTSP_SENSOR_CHN,
            alignment=12,
            crop=None,
        )
        self.sensor.set_pixformat(Sensor.YUV420SP, chn=RTSP_SENSOR_CHN)
        self._verify_output_geometry()

    def _verify_output_geometry(self):
        try:
            actual_width = int(self.sensor.width(chn=RTSP_SENSOR_CHN))
            actual_height = int(self.sensor.height(chn=RTSP_SENSOR_CHN))
        except BaseException as error:
            raise RuntimeError("无法读取 RTSP Sensor 通道尺寸：%s" % error)

        if actual_width != RTSP_OUTPUT_WIDTH or actual_height != RTSP_OUTPUT_HEIGHT:
            raise RuntimeError(
                "RTSP 通道实际输出为 %dx%d，不是要求的 %dx%d" % (
                    actual_width,
                    actual_height,
                    RTSP_OUTPUT_WIDTH,
                    RTSP_OUTPUT_HEIGHT,
                )
            )

        try:
            crop_enabled = bool(self.sensor._chn_attr[RTSP_SENSOR_CHN].crop_enable)
        except BaseException as error:
            raise RuntimeError("无法确认 RTSP 通道 crop_enable：%s" % error)

        if crop_enabled:
            raise RuntimeError("RTSP Sensor 通道启用了裁剪；拒绝继续推流")
        print("RTSP_OUTPUT=%dx%d CHN=%d CROP=off" % (
            actual_width, actual_height, RTSP_SENSOR_CHN
        ))

    def _create_encoder_attr(self):
        try:
            return ChnAttrStr(
                self.encoder.PAYLOAD_TYPE_H264,
                self.encoder.H264_PROFILE_MAIN,
                RTSP_OUTPUT_WIDTH,
                RTSP_OUTPUT_HEIGHT,
                bit_rate=RTSP_BITRATE_KBPS,
                gopLen=RTSP_GOP_LENGTH,
                src_frame_rate=RTSP_TARGET_FPS,
                dst_frame_rate=RTSP_TARGET_FPS,
            )
        except TypeError as error:
            raise RuntimeError("ChnAttrStr 与 CanMV v1.8 签名不一致：%s" % error)

    def prepare(self):
        """在 MediaManager.init/sensor.run 前申请 VENC、绑定 CH1 并启动 RTSP 服务。"""
        self.encoder = Encoder()
        self._require_success(
            "Encoder.SetOutBufs",
            self.encoder.SetOutBufs(
                self.venc_chn,
                RTSP_VENC_BUFFER_COUNT,
                RTSP_OUTPUT_WIDTH,
                RTSP_OUTPUT_HEIGHT,
            ),
        )
        self.link = MediaManager.link(
            self.sensor.bind_info(chn=RTSP_SENSOR_CHN)["src"],
            (VIDEO_ENCODE_MOD_ID, VENC_DEV_ID, self.venc_chn),
        )
        self._require_success(
            "Encoder.Create",
            self.encoder.Create(self.venc_chn, self._create_encoder_attr()),
        )
        self._encoder_created = True

        self.rtspserver = mm.rtsp_server()
        self._require_success(
            "RTSP init", self.rtspserver.rtspserver_init(RTSP_PORT)
        )
        self._rtsp_initialized = True
        self._require_success(
            "RTSP createsession",
            self.rtspserver.rtspserver_createsession(
                RTSP_SESSION,
                mm.multi_media_type.media_h264,
                False,
            ),
        )
        self._require_success("RTSP start", self.rtspserver.rtspserver_start())
        self._rtsp_started = True
        print("VENC 配置: %dx%d@%d H.264 Main GOP=%d bitrate=%dkb/s buffers=%d" % (
            RTSP_OUTPUT_WIDTH,
            RTSP_OUTPUT_HEIGHT,
            RTSP_TARGET_FPS,
            RTSP_GOP_LENGTH,
            RTSP_BITRATE_KBPS,
            RTSP_VENC_BUFFER_COUNT,
        ))

    def start_encoder(self):
        self._require_success("Encoder.Start", self.encoder.Start(self.venc_chn))
        self._encoder_started = True

    def start_after_sensor_run(self):
        _rtsp_verify_actual_acquisition_mode(self.sensor, "after_run")
        self._verify_output_geometry()

        self.running = True
        self._stream_start_ms = _rtsp_ticks_ms()
        self._last_report_ms = self._stream_start_ms
        self._last_report_frames = self.total_frames
        self._last_report_encoded_bytes = self.total_encoded_bytes
        self._last_report_rtsp_bytes = self.total_rtsp_bytes
        self._last_report_pts_us = None
        self._last_report_valid_pts_frames = self._valid_pts_frames
        self._send_max_ms_report = 0.0
        self._thread_done = False
        _thread.start_new_thread(self._stream_loop, ())
        self._thread_started = True

    def get_rtsp_url(self, ip_address):
        return "rtsp://%s:%d/%s" % (ip_address, RTSP_PORT, RTSP_SESSION)

    def _get_stream(self, stream_data):
        try:
            return self.encoder.GetStream(
                self.venc_chn,
                stream_data,
                timeout=RTSP_GET_STREAM_TIMEOUT_MS,
            )
        except TypeError:
            try:
                return self.encoder.GetStream(
                    self.venc_chn,
                    stream_data,
                    RTSP_GET_STREAM_TIMEOUT_MS,
                )
            except TypeError:
                return self.encoder.GetStream(self.venc_chn, stream_data)

    def _rtsp_timestamp(self, pts_us):
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

    def _record_frame_pts(self, pts_us):
        try:
            pts_us = int(pts_us)
        except (TypeError, ValueError):
            return
        if pts_us <= 0:
            return
        if self._last_frame_pts_us is not None and pts_us <= self._last_frame_pts_us:
            return

        wall_now_ms = _rtsp_ticks_ms()
        if self._first_frame_pts_us is None:
            self._first_frame_pts_us = pts_us
            self._pts_origin_us = pts_us
            self._pts_wall_origin_ms = wall_now_ms
            self.queue_drift_ms = 0.0

        self._last_frame_pts_us = pts_us
        self._valid_pts_frames += 1
        wall_elapsed_ms = _rtsp_ticks_diff(wall_now_ms, self._pts_wall_origin_ms)
        pts_elapsed_ms = (pts_us - self._pts_origin_us) / 1000.0
        self.queue_drift_ms = wall_elapsed_ms - pts_elapsed_ms

    def _check_startup_fps_gate(self, now_ms):
        if self._startup_gate_checked or self._stream_start_ms is None:
            return True
        elapsed_ms = _rtsp_ticks_diff(now_ms, self._stream_start_ms)
        if elapsed_ms < RTSP_STARTUP_GATE_MS:
            return True

        self._startup_gate_checked = True
        startup_fps = self.total_frames * 1000.0 / elapsed_ms
        print("[RTSP_STARTUP] elapsed_ms=%d frames=%d avg_fps=%.1f allowed=%.1f..%.1f" % (
            elapsed_ms,
            self.total_frames,
            startup_fps,
            RTSP_STARTUP_FPS_MIN,
            RTSP_STARTUP_FPS_MAX,
        ))
        if startup_fps < RTSP_STARTUP_FPS_MIN or startup_fps > RTSP_STARTUP_FPS_MAX:
            self.fatal_error = RuntimeError(
                "RTSP_STARTUP_FPS_GATE_FAIL: %.1f fps 不在 %.1f..%.1f" % (
                    startup_fps,
                    RTSP_STARTUP_FPS_MIN,
                    RTSP_STARTUP_FPS_MAX,
                )
            )
            self.running = False
            print("[RTSP_FATAL]", self.fatal_error)
            return False

        print("RTSP_STARTUP_FPS_GATE=PASS")
        return True

    def _stream_loop(self):
        stream_data = StreamData()
        try:
            while self.running:
                _rtsp_exitpoint()
                stream_acquired = False
                try:
                    try:
                        result = self._get_stream(stream_data)
                    except BaseException as error:
                        self.getstream_errors += 1
                        self.buffer_fallback_required = (
                            RTSP_VENC_BUFFER_COUNT == 4
                        )
                        raise RuntimeError(
                            "GetStream 调用异常；buffers=%d，回退值=%d，原始错误=%s" % (
                                RTSP_VENC_BUFFER_COUNT,
                                RTSP_VENC_BUFFER_FALLBACK_COUNT,
                                error,
                            )
                        )

                    if result not in (None, 0):
                        self.getstream_errors += 1
                        self.buffer_fallback_required = (
                            RTSP_VENC_BUFFER_COUNT == 4
                        )
                        raise RuntimeError(
                            "GetStream 失败，返回码=%s，buffers=%d，回退值=%d" % (
                                result,
                                RTSP_VENC_BUFFER_COUNT,
                                RTSP_VENC_BUFFER_FALLBACK_COUNT,
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
                            raise RuntimeError(
                                "非法 VENC pack: index=%d size=%s addr=%s" % (
                                    pack_idx, packet_size, packet_addr
                                )
                            )

                        self.total_packs += 1
                        self.total_encoded_bytes += packet_size
                        try:
                            packet = bytes(uctypes.bytearray_at(packet_addr, packet_size))
                            send_begin_us = _rtsp_ticks_us()
                            try:
                                send_result = self.rtspserver.rtspserver_sendvideodata(
                                    RTSP_SESSION,
                                    packet,
                                    packet_size,
                                    self._rtsp_timestamp(stream_data.pts[pack_idx]),
                                )
                            finally:
                                send_elapsed_ms = _rtsp_ticks_us_diff(
                                    _rtsp_ticks_us(), send_begin_us
                                ) / 1000.0
                                if send_elapsed_ms > self._send_max_ms_report:
                                    self._send_max_ms_report = send_elapsed_ms
                            self._require_success("RTSP sendvideodata", send_result)
                            self.total_rtsp_bytes += packet_size
                        except BaseException as error:
                            self.rtsp_errors += 1
                            if self.rtsp_errors == 1 or self.rtsp_errors % 30 == 0:
                                _rtsp_print_exception("RTSP 发送异常:", error)
                except BaseException as error:
                    self.venc_errors += 1
                    self.fatal_error = error
                    _rtsp_print_exception("VENC GetStream/处理异常:", error)
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
                            _rtsp_print_exception("VENC ReleaseStream 异常:", error)
                            self.running = False
        finally:
            self._thread_done = True

    def report(self, wlan):
        now = _rtsp_ticks_ms()
        if self.running and wlan is not None and not wlan.isconnected():
            self.fatal_error = RuntimeError("Wi-Fi 已断开，RTSP 编码线程停止")
            self.running = False

        elapsed_ms = _rtsp_ticks_diff(now, self._last_report_ms)
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
                pts_frame_delta = current_pts_frames - self._last_report_valid_pts_frames
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

        print("[RTSP_STAT] enc_fps=%.1f pts_fps=%.1f drift_ms=%.1f "
              "send_max_ms=%.1f enc=%.0fkb/s rtsp=%.0fkb/s "
              "venc_err=%d get_err=%d rel_err=%d rtsp_err=%d "
              "pts_zero=%d pts_back=%d rssi=%sdBm" % (
                  fps,
                  pts_fps,
                  self.queue_drift_ms,
                  self._send_max_ms_report,
                  encoded_kbps,
                  rtsp_kbps,
                  self.venc_errors,
                  self.getstream_errors,
                  self.release_errors,
                  self.rtsp_errors,
                  self.pts_zero,
                  self.pts_regressions,
                  _rtsp_read_rssi(wlan),
              ))

        self._last_report_ms = now
        self._last_report_frames = self.total_frames
        self._last_report_encoded_bytes = self.total_encoded_bytes
        self._last_report_rtsp_bytes = self.total_rtsp_bytes
        self._last_report_pts_us = current_pts_us
        self._last_report_valid_pts_frames = current_pts_frames
        self._send_max_ms_report = 0.0
        self._check_startup_fps_gate(now)

        if self.fatal_error is not None and not self._fatal_reported:
            self._fatal_reported = True
            print("RTSP 编码线程已停止:", self.fatal_error)
            if self.buffer_fallback_required:
                print("RTSP_BUFFER_FALLBACK_REQUIRED=%d" % RTSP_VENC_BUFFER_FALLBACK_COUNT)

    def request_stop_and_wait(self, timeout_ms=RTSP_STOP_WAIT_MS):
        self.running = False
        if self._thread_started and not self._thread_done:
            stop_begin = _rtsp_ticks_ms()
            while not self._thread_done:
                if _rtsp_ticks_diff(_rtsp_ticks_ms(), stop_begin) >= timeout_ms:
                    print("RTSP 编码线程未在 %dms 内退出。" % timeout_ms)
                    return False
                _rtsp_sleep_ms(20)
        return True

    def teardown_after_sensor_stop(self):
        """必须在 07 已停止共享 Sensor 后调用，避免与 GetStream/ReleaseStream 竞争。"""
        if self._cleanup_complete:
            return True
        if self._thread_started and not self._thread_done:
            print("RTSP 线程仍在运行；为避免并发释放，本次不强制销毁 VENC/RTSP。")
            return False

        if self.link is not None:
            try:
                if hasattr(self.link, "destroy"):
                    self.link.destroy()
            except BaseException as error:
                _rtsp_print_exception("Media link 释放异常:", error)
            self.link = None

        if self._encoder_started and self.encoder:
            try:
                self.encoder.Stop(self.venc_chn)
            except BaseException as error:
                _rtsp_print_exception("VENC stop 异常:", error)
            self._encoder_started = False

        if self._encoder_created and self.encoder:
            try:
                self.encoder.Destroy(self.venc_chn)
            except BaseException as error:
                _rtsp_print_exception("VENC destroy 异常:", error)
            self._encoder_created = False

        if self._rtsp_started:
            try:
                self.rtspserver.rtspserver_stop()
            except BaseException as error:
                _rtsp_print_exception("RTSP stop 异常:", error)
            self._rtsp_started = False

        if self._rtsp_initialized:
            try:
                self.rtspserver.rtspserver_deinit()
            except BaseException as error:
                _rtsp_print_exception("RTSP deinit 异常:", error)
            self._rtsp_initialized = False

        self._cleanup_complete = True
        return True


def _rtsp_optional_failure(prefix, error):
    _rtsp_print_exception(prefix, error)
    if RTSP_REQUIRED:
        raise error
    print("RTSP_DISABLED_CONTINUE_07")

# ===================== END 03 RTSP INTEGRATION =====================
fpioa = FPIOA()
# UART2: 参考 06串口通信.py，TX=11，RX=12
fpioa.set_function(11, FPIOA.UART2_TXD)
fpioa.set_function(12, FPIOA.UART2_RXD)
# M0 侧 UART0 配置为 115200，8N1
uart2 = UART(UART.UART2, 115200)

LOST_DX = -999

def send_dx_to_m0(dx):
    """按 M0 uart.c 协议发送 @dx\r\n"""
    uart2.write("@" + str(int(dx)) + "\r\n")

display_mode="lcd"
if display_mode=="lcd":
    DISPLAY_WIDTH = ALIGN_UP(800, 16)
    DISPLAY_HEIGHT = 480
else:
    DISPLAY_WIDTH = ALIGN_UP(1920, 16)
    DISPLAY_HEIGHT = 1080

OUT_RGB888P_WIDTH = ALIGN_UP(1080, 16)
OUT_RGB888P_HEIGH = 720

# 颜色盘
color_four = [(255, 220, 20, 60), (255, 119, 11, 32), (255, 0, 0, 142), (255, 0, 0, 230),
        (255, 106, 0, 228), (255, 0, 60, 100), (255, 0, 80, 100), (255, 0, 0, 70),
        (255, 0, 0, 192), (255, 250, 170, 30), (255, 100, 170, 30), (255, 220, 220, 0),
        (255, 175, 116, 175), (255, 250, 0, 30), (255, 165, 42, 42), (255, 255, 77, 255),
        (255, 0, 226, 252), (255, 182, 182, 255), (255, 0, 82, 0), (255, 120, 166, 157),
        (255, 110, 76, 0), (255, 174, 57, 255), (255, 199, 100, 0), (255, 72, 0, 118),
        (255, 255, 179, 240), (255, 0, 125, 92), (255, 209, 0, 151), (255, 188, 208, 182),
        (255, 0, 220, 176), (255, 255, 99, 164), (255, 92, 0, 73), (255, 133, 129, 255),
        (255, 78, 180, 255), (255, 0, 228, 0), (255, 174, 255, 243), (255, 45, 89, 255),
        (255, 134, 134, 103), (255, 145, 148, 174), (255, 255, 208, 186),
        (255, 197, 226, 255), (255, 171, 134, 1), (255, 109, 63, 54), (255, 207, 138, 255),
        (255, 151, 0, 95), (255, 9, 80, 61), (255, 84, 105, 51), (255, 74, 65, 105),
        (255, 166, 196, 102), (255, 208, 195, 210), (255, 255, 109, 65), (255, 0, 143, 149),
        (255, 179, 0, 194), (255, 209, 99, 106), (255, 5, 121, 0), (255, 227, 255, 205),
        (255, 147, 186, 208), (255, 153, 69, 1), (255, 3, 95, 161), (255, 163, 255, 0),
        (255, 119, 0, 170), (255, 0, 182, 199), (255, 0, 165, 120), (255, 183, 130, 88),
        (255, 95, 32, 0), (255, 130, 114, 135), (255, 110, 129, 133), (255, 166, 74, 118),
        (255, 219, 142, 185), (255, 79, 210, 114), (255, 178, 90, 62), (255, 65, 70, 15),
        (255, 127, 167, 115), (255, 59, 105, 106), (255, 142, 108, 45), (255, 196, 172, 0),
        (255, 95, 54, 80), (255, 128, 76, 255), (255, 201, 57, 1), (255, 246, 0, 122),
        (255, 191, 162, 208)]

root_path="/sdcard/mp_deployment_source/"
config_path=root_path+"deploy_config.json"
deploy_conf={}
debug_mode=1

class ScopedTiming:
    def __init__(self, info="", enable_profile=True):
        self.info = info
        self.enable_profile = enable_profile

    def __enter__(self):
        if self.enable_profile:
            self.start_time = time.time_ns()
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        if self.enable_profile:
            elapsed_time = time.time_ns() - self.start_time
            print(f"{self.info} took {elapsed_time / 1000000:.2f} ms")

def read_deploy_config(config_path):
    # 打开JSON文件以进行读取deploy_config
    with open(config_path, 'r') as json_file:
        try:
            # 从文件中加载JSON数据
            config = ujson.load(json_file)

            # 打印数据（可根据需要执行其他操作）
            #print(config)
        except ValueError as e:
            print("JSON 解析错误:", e)
    return config

# 重叠检测框合并阈值（IoU > 此值则合并）
MERGE_IOU_THRESH = 0.3

def merge_overlap_boxes(boxes):
    """合并重叠的检测框（IoU > 阈值则合并为一个外接框）"""
    if len(boxes) <= 1:
        return boxes
    merged = []
    used = [False] * len(boxes)
    for i in range(len(boxes)):
        if used[i]:
            continue
        # 取当前框作为起始
        x1, y1, x2, y2 = boxes[i][2], boxes[i][3], boxes[i][4], boxes[i][5]
        best_conf = boxes[i][1]
        best_cls = boxes[i][0]
        used[i] = True
        changed = True
        while changed:
            changed = False
            for j in range(len(boxes)):
                if used[j]:
                    continue
                bx1, by1, bx2, by2 = boxes[j][2], boxes[j][3], boxes[j][4], boxes[j][5]
                # 计算IoU
                ix1 = max(x1, bx1); iy1 = max(y1, by1)
                ix2 = min(x2, bx2); iy2 = min(y2, by2)
                inter = max(0, ix2 - ix1) * max(0, iy2 - iy1)
                area_a = (x2 - x1) * (y2 - y1)
                area_b = (bx2 - bx1) * (by2 - by1)
                union = area_a + area_b - inter
                iou = inter / union if union > 0 else 0
                if iou > MERGE_IOU_THRESH:
                    # 合并：扩大外接框
                    x1 = min(x1, bx1); y1 = min(y1, by1)
                    x2 = max(x2, bx2); y2 = max(y2, by2)
                    if boxes[j][1] > best_conf:
                        best_conf = boxes[j][1]
                        best_cls = boxes[j][0]
                    used[j] = True
                    changed = True
        merged.append([best_cls, best_conf, x1, y1, x2, y2])
    return merged

def detection():
    print("det_infer start")
    # ================= BEGIN 03 RTSP HOOK: NETWORK =================
    rtsp_wlan = None
    rtsp_streamer = None
    if RTSP_ENABLED:
        print("RTSP_PROGRAM_VERSION=" + RTSP_PROGRAM_VERSION)
        try:
            _rtsp_require_sensor_mode_available()
            rtsp_wlan = connect_wifi_for_rtsp()
            if rtsp_wlan is None:
                raise RuntimeError("Wi-Fi 未连接，RTSP_NOT_READY")
        except BaseException as error:
            rtsp_wlan = None
            _rtsp_optional_failure("RTSP 网络/模式检查失败:", error)
    # ================== END 03 RTSP HOOK: NETWORK ==================
    # 使用json读取内容初始化部署变量
    deploy_conf=read_deploy_config(config_path)
    kmodel_name=deploy_conf["kmodel_path"]
    labels=deploy_conf["categories"]
    confidence_threshold= deploy_conf["confidence_threshold"]
    nms_threshold = deploy_conf["nms_threshold"]
    img_size=deploy_conf["img_size"]
    num_classes=deploy_conf["num_classes"]
    nms_option = deploy_conf["nms_option"]
    model_type = deploy_conf["model_type"]
    if model_type == "AnchorBaseDet":
        anchors = deploy_conf["anchors"][0] + deploy_conf["anchors"][1] + deploy_conf["anchors"][2]
    kmodel_frame_size = img_size
    frame_size = [OUT_RGB888P_WIDTH,OUT_RGB888P_HEIGH]
    strides = [8,16,32]

    # 计算padding值
    ori_w = OUT_RGB888P_WIDTH;
    ori_h = OUT_RGB888P_HEIGH;
    width = kmodel_frame_size[0];
    height = kmodel_frame_size[1];
    ratiow = float(width) / ori_w;
    ratioh = float(height) / ori_h;
    if ratiow < ratioh:
        ratio = ratiow
    else:
        ratio = ratioh
    new_w = int(ratio * ori_w);
    new_h = int(ratio * ori_h);
    dw = float(width - new_w) / 2;
    dh = float(height - new_h) / 2;
    top = int(round(dh - 0.1));
    bottom = int(round(dh + 0.1));
    left = int(round(dw - 0.1));
    right = int(round(dw - 0.1));

    # init kpu and load kmodel
    kpu = nn.kpu()
    ai2d = nn.ai2d()
    kpu.load_kmodel(root_path+kmodel_name)
    ai2d.set_dtype(nn.ai2d_format.NCHW_FMT,
                                   nn.ai2d_format.NCHW_FMT,
                                   np.uint8, np.uint8)
    ai2d.set_pad_param(True, [0,0,0,0,top,bottom,left,right], 0, [114,114,114])
    ai2d.set_resize_param(True, nn.interp_method.tf_bilinear, nn.interp_mode.half_pixel )
    ai2d_builder = ai2d.build([1,3,OUT_RGB888P_HEIGH,OUT_RGB888P_WIDTH], [1,3,height,width])

    # 初始化并配置sensor
    # ============ BEGIN 03 RTSP HOOK: FORCE 1080P30 MODE ============
    # 本板 CanMV v1.8 的 Sensor() 默认选择 1920x1080@60；
    # RTSP 启用时沿用 03 的显式构造参数，确保真实采集为 1080P30。
    if rtsp_wlan is not None:
        sensor = Sensor(
            width=RTSP_SENSOR_MODE_WIDTH,
            height=RTSP_SENSOR_MODE_HEIGHT,
            fps=RTSP_SENSOR_MODE_FPS,
        )
    else:
        sensor = Sensor()
    # ============= END 03 RTSP HOOK: FORCE 1080P30 MODE =============
    sensor.reset()
    # ============== BEGIN 03 RTSP HOOK: SHARED SENSOR ==============
    if rtsp_wlan is not None:
        try:
            _rtsp_verify_actual_acquisition_mode(sensor, "after_reset")
            rtsp_streamer = SharedSensorH264RtspStreamer(sensor)
            rtsp_streamer.configure_sensor_channel()
        except BaseException as error:
            rtsp_streamer = None
            _rtsp_optional_failure("RTSP Sensor CH1 配置失败:", error)
    # =============== END 03 RTSP HOOK: SHARED SENSOR ===============
    # 设置镜像
    sensor.set_hmirror(False)
    # 设置翻转
    sensor.set_vflip(False)
    # 通道0直接给到显示VO，格式为YUV420
    sensor.set_framesize(width = DISPLAY_WIDTH, height = DISPLAY_HEIGHT)
    sensor.set_pixformat(PIXEL_FORMAT_YUV_SEMIPLANAR_420)
    # 通道2给到AI做算法处理，格式为RGB888
    sensor.set_framesize(width = OUT_RGB888P_WIDTH , height = OUT_RGB888P_HEIGH, chn=CAM_CHN_ID_2)
    sensor.set_pixformat(PIXEL_FORMAT_RGB_888_PLANAR, chn=CAM_CHN_ID_2)
    # 绑定通道0的输出到vo
    sensor_bind_info = sensor.bind_info(x = 0, y = 0, chn = CAM_CHN_ID_0)
    Display.bind_layer(**sensor_bind_info, layer = Display.LAYER_VIDEO1)
    if display_mode=="lcd":
        # 设置为ST7701显示，默认800x480
        Display.init(Display.ST7701, to_ide = True)
    else:
        # 设置为LT9611显示，默认1920x1080
        Display.init(Display.LT9611, to_ide = True)

    # =============== BEGIN 03 RTSP HOOK: PREPARE VENC ===============
    if rtsp_streamer is not None:
        try:
            rtsp_streamer.prepare()
        except BaseException as error:
            failed_streamer = rtsp_streamer
            rtsp_streamer = None
            failed_streamer.request_stop_and_wait()
            failed_streamer.teardown_after_sensor_stop()
            _rtsp_optional_failure("RTSP VENC/服务初始化失败:", error)
    # ================ END 03 RTSP HOOK: PREPARE VENC ================
    #创建OSD图像
    osd_img = image.Image(DISPLAY_WIDTH, DISPLAY_HEIGHT, image.ARGB8888)
    try:
        # media初始化
        MediaManager.init()
        # ============== BEGIN 03 RTSP HOOK: START VENC ==============
        if rtsp_streamer is not None:
            try:
                rtsp_streamer.start_encoder()
            except BaseException as error:
                failed_streamer = rtsp_streamer
                rtsp_streamer = None
                failed_streamer.request_stop_and_wait()
                failed_streamer.teardown_after_sensor_stop()
                _rtsp_optional_failure("RTSP Encoder.Start 失败:", error)
        # =============== END 03 RTSP HOOK: START VENC ===============
        # 启动sensor
        sensor.run()
        # ============== BEGIN 03 RTSP HOOK: START STREAM =============
        if rtsp_streamer is not None:
            try:
                rtsp_streamer.start_after_sensor_run()
                rtsp_url = rtsp_streamer.get_rtsp_url(rtsp_wlan.ifconfig()[0])
                print("RTSP_READY=" + rtsp_url)
                print("把 RTSP_READY= 后的完整数字 IP 地址粘贴到 VLC。")
            except BaseException as error:
                rtsp_streamer.request_stop_and_wait()
                _rtsp_optional_failure("RTSP 编码线程启动失败:", error)
        # =============== END 03 RTSP HOOK: START STREAM ==============
        rgb888p_img = None
        ai2d_input_tensor = None
        data = np.ones((1,3,width,height),dtype=np.uint8)
        ai2d_output_tensor = nn.from_numpy(data)
        # FPS 计时变量
        fps = 0
        filtered_dx = 0          # 一阶低通滤波后的dx
        consecutive_jumps = 0    # 连续跳变超过限幅的帧数
        jump_raw_dx_list = []    # 连续跳变期间的raw_dx记录
        CENTER_X_AI = OUT_RGB888P_WIDTH / 2   # AI坐标系中心X = 540
        last_ticks = time.ticks_ms()
        while  True:
            with ScopedTiming("total",debug_mode > 0):
                rgb888p_img = sensor.snapshot(chn=CAM_CHN_ID_2)
                # for rgb888planar
                if rgb888p_img.format() == image.RGBP888:
                    ai2d_input = rgb888p_img.to_numpy_ref()
                    ai2d_input_tensor = nn.from_numpy(ai2d_input)
                    ai2d_builder.run(ai2d_input_tensor, ai2d_output_tensor)

                    # set input
                    kpu.set_input_tensor(0, ai2d_output_tensor)
                    # run kmodel
                    kpu.run()
                    # get output
                    results = []
                    for i in range(kpu.outputs_size()):
                        out_data = kpu.get_output_tensor(i)
                        result = out_data.to_numpy()
                        result = result.reshape((result.shape[0]*result.shape[1]*result.shape[2]*result.shape[3]))
                        del out_data
                        results.append(result)
                    gc.collect()

                    # postprocess
                    if model_type == "AnchorBaseDet":
                        det_boxes = aicube.anchorbasedet_post_process( results[0], results[1], results[2], kmodel_frame_size, frame_size, strides, num_classes, confidence_threshold, nms_threshold, anchors, nms_option)
                    elif model_type == "GFLDet":
                        det_boxes = aicube.gfldet_post_process( results[0], results[1], results[2], kmodel_frame_size, frame_size, strides, num_classes, confidence_threshold, nms_threshold, nms_option)
                    else:
                        det_boxes = aicube.anchorfreedet_post_process( results[0], results[1], results[2], kmodel_frame_size, frame_size, strides, num_classes, confidence_threshold, nms_threshold, nms_option)
                    # 合并重叠的检测框
                    det_boxes = merge_overlap_boxes(det_boxes)
                    osd_img.clear()
                    # 屏幕中心画直径4像素的绿点
                    osd_img.draw_circle(DISPLAY_WIDTH // 2, DISPLAY_HEIGHT // 2, 2, color=(0, 255, 0), fill=True)
                    # 寻找面积最大的检测框，计算其中心坐标 (ax, ay)
                    ax = -1
                    ay = -1
                    max_area = 0
                    if det_boxes:
                        for det_boxe in det_boxes:
                            x1, y1, x2, y2 = det_boxe[2],det_boxe[3],det_boxe[4],det_boxe[5]
                            w = float(x2 - x1) * DISPLAY_WIDTH // OUT_RGB888P_WIDTH
                            h = float(y2 - y1) * DISPLAY_HEIGHT // OUT_RGB888P_HEIGH
                            osd_img.draw_rectangle(int(x1 * DISPLAY_WIDTH // OUT_RGB888P_WIDTH) , int(y1 * DISPLAY_HEIGHT // OUT_RGB888P_HEIGH) , int(w) , int(h) , color=color_four[det_boxe[0]][1:])
                            label = labels[det_boxe[0]]
                            score = str(round(det_boxe[1],2))
                            osd_img.draw_string_advanced( int(x1 * DISPLAY_WIDTH // OUT_RGB888P_WIDTH) , int(y1 * DISPLAY_HEIGHT // OUT_RGB888P_HEIGH)-50,32, label + " " + score , color=color_four[det_boxe[0]][1:])
                            # 找面积最大的框
                            area = w * h
                            if area > max_area:
                                max_area = area
                                ax = (x1 + x2) / 2  # 最大框的中心X（AI坐标系）
                                ay = (y1 + y2) / 2  # 最大框的中心Y（AI坐标系）
                    if ax >= 0:
                        # 计算原始dx（正=右，负=左）
                        raw_dx = ax - CENTER_X_AI
                        # 限幅+防死区：跳变超过300像素
                        if abs(raw_dx - filtered_dx) > 300:
                            consecutive_jumps += 1
                            jump_raw_dx_list.append(raw_dx)
                            # 连续10帧跳变且期间ax波动不超过300 → 球真的移动了，接受
                            if consecutive_jumps >= 10:
                                if max(jump_raw_dx_list) - min(jump_raw_dx_list) <= 300:
                                    filtered_dx = raw_dx  # 直接更新到新位置
                                consecutive_jumps = 0
                                jump_raw_dx_list = []
                            # 否则丢弃，保持旧值
                        else:
                            # 正常范围内：重置计数器，一阶低通滤波
                            consecutive_jumps = 0
                            jump_raw_dx_list = []
                            filtered_dx = filtered_dx * 0.7 + raw_dx * 0.3
                        # 串口2按 M0 协议发送 filtered_dx
                        send_dx_to_m0(int(filtered_dx))
                        # 映射到LCD坐标系
                        lcd_ax = int(ax * DISPLAY_WIDTH // OUT_RGB888P_WIDTH)
                        lcd_ay = int(ay * DISPLAY_HEIGHT // OUT_RGB888P_HEIGH)
                        osd_img.draw_string_advanced(2, 30, 24, "ax=" + str(int(ax)), color=(0, 255, 0))
                        osd_img.draw_string_advanced(2, 56, 24, "dx=" + str(int(filtered_dx)), color=(255, 0, 0))
                        # 绿色细线1：(ax, 中心Y) → 屏幕中心    （水平线）
                        osd_img.draw_line(lcd_ax, DISPLAY_HEIGHT // 2, DISPLAY_WIDTH // 2, DISPLAY_HEIGHT // 2, color=(0, 255, 0), thickness=1)
                        # 绿色细线2：(ax, ay) → (ax, 中心Y)    （竖直线）
                        osd_img.draw_line(lcd_ax, lcd_ay, lcd_ax, DISPLAY_HEIGHT // 2, color=(0, 255, 0), thickness=1)
                        # 红色细线：filtered_dx映射到LCD → 屏幕中心（比中心Y低4像素）
                        lcd_fdx = int(filtered_dx * DISPLAY_WIDTH // OUT_RGB888P_WIDTH)
                        red_y = DISPLAY_HEIGHT // 2 + 4
                        osd_img.draw_line(DISPLAY_WIDTH // 2 + lcd_fdx, red_y, DISPLAY_WIDTH // 2, red_y, color=(255, 0, 0), thickness=3)
                    else:
                        # 丢球/未检测到时发送显式丢失状态
                        send_dx_to_m0(LOST_DX)
                    # 计算并显示实时FPS（左上角）
                    now = time.ticks_ms()
                    dt = time.ticks_diff(now, last_ticks) / 1000.0  # 毫秒转秒
                    last_ticks = now
                    if dt > 0:
                        fps = 0.9 * fps + 0.1 * (1.0 / dt)  # 指数平滑
                    osd_img.draw_string_advanced(2, 2, 24, "FPS=" + str(int(fps)), color=(0, 255, 255))
                    Display.show_image(osd_img, 0, 0, Display.LAYER_OSD3)
                    gc.collect()
                # =============== BEGIN 03 RTSP HOOK: REPORT ===============
                if (rtsp_streamer is not None and
                        (rtsp_streamer.running or not rtsp_streamer._fatal_reported)):
                    rtsp_streamer.report(rtsp_wlan)
                # ================ END 03 RTSP HOOK: REPORT ================
                rgb888p_img = None
    except Exception as e:
        print(f"An error occurred during buffer used: {e}")
    finally:
        os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
        del ai2d_input_tensor
        del ai2d_output_tensor
        # ============ BEGIN 03 RTSP HOOK: QUIESCE THREAD ============
        rtsp_thread_stopped = True
        if rtsp_streamer is not None:
            rtsp_thread_stopped = rtsp_streamer.request_stop_and_wait()
        # ============= END 03 RTSP HOOK: QUIESCE THREAD =============
        #停止摄像头输出
        sensor.stop()
        # =============== BEGIN 03 RTSP HOOK: TEARDOWN ===============
        if rtsp_streamer is not None:
            if not rtsp_thread_stopped:
                rtsp_streamer.request_stop_and_wait(1000)
            if not rtsp_streamer.teardown_after_sensor_stop():
                print("RTSP 资源未强制释放；请重启 K230 后再运行。")
            else:
                print("RTSP 图传已停止。")
        # ================ END 03 RTSP HOOK: TEARDOWN ================
        #去初始化显示设备
        Display.deinit()
        #释放媒体缓冲区
        MediaManager.deinit()
        gc.collect()
        time.sleep(1)
        nn.shrink_memory_pool()
    print("det_infer end")
    return 0


if __name__=="__main__":
    detection()
