# K230 双轴云台绿色色块追踪
# 目标：
# 1. 摄像头画面上下翻转
# 2. 识别绿色色块并绘制绿色十字
# 3. 通过 IO5/IO6、UART2、115200 向 STM32 发送：
#       X:<cx>,Y:<cy>,W:<w>,H:<h>\n
#    未检测到目标时发送：
#       NO_TARGET\n
#
# 适用：CanMV K230 新版 media.sensor / media.display API
# 屏幕示例：800x480 ST7701
#
# 接线：
# K230 IO5 / UART2_TX -> STM32 PA3 / USART2_RX
# K230 IO6 / UART2_RX <- STM32 PA2 / USART2_TX
# K230 GND            <-> STM32 GND

import time
import os
import gc

from machine import FPIOA, UART
from media.sensor import *
from media.display import *
from media.media import *

# ==================== 用户配置 ====================

IMAGE_WIDTH = 800
IMAGE_HEIGHT = 480

# 800x480 MIPI LCD。若你原工程使用其他屏幕类型，只替换这一项。
DISPLAY_TYPE = Display.ST7701

# 画面方向
ENABLE_VERTICAL_FLIP = True
ENABLE_HORIZONTAL_MIRROR = False

# 绿色 LAB 阈值：需要根据实际灯光在 CanMV IDE 阈值编辑器中微调
# 格式：(L_min, L_max, A_min, A_max, B_min, B_max)
GREEN_THRESHOLDS = [
    (15, 100, -80, -10, -10, 80)
]

# 过滤小噪点
PIXELS_THRESHOLD = 250
AREA_THRESHOLD = 250
MERGE_MARGIN = 10

# UART：IO5=TX，IO6=RX
UART_BAUDRATE = 115200

# 每隔多少毫秒至少发送一次状态。设为 0 表示每帧发送。
SEND_INTERVAL_MS = 0

# ==================== 全局对象 ====================

sensor = None
uart = None


def init_uart():
    """初始化 K230 UART2：IO5 TX、IO6 RX。"""
    global uart

    fpioa = FPIOA()
    fpioa.set_function(5, FPIOA.UART2_TXD)
    fpioa.set_function(6, FPIOA.UART2_RXD)

    uart = UART(
        UART.UART2,
        baudrate=UART_BAUDRATE,
        bits=UART.EIGHTBITS,
        parity=UART.PARITY_NONE,
        stop=UART.STOPBITS_ONE
    )


def init_camera_and_display():
    """初始化摄像头和 800x480 LCD。"""
    global sensor

    sensor = Sensor()
    sensor.reset()

    # 上下翻转必须在 sensor.run() 之前设置
    sensor.set_vflip(ENABLE_VERTICAL_FLIP)
    sensor.set_hmirror(ENABLE_HORIZONTAL_MIRROR)

    sensor.set_framesize(
        width=IMAGE_WIDTH,
        height=IMAGE_HEIGHT
    )
    sensor.set_pixformat(Sensor.RGB565)

    Display.init(
        DISPLAY_TYPE,
        width=IMAGE_WIDTH,
        height=IMAGE_HEIGHT,
        to_ide=True
    )

    MediaManager.init()
    sensor.run()


def select_largest_blob(blobs):
    """从候选色块中选择像素数最多的一个。"""
    best_blob = None
    best_pixels = 0

    for blob in blobs:
        pixels = blob.pixels()
        if pixels > best_pixels:
            best_pixels = pixels
            best_blob = blob

    return best_blob


def send_target(blob):
    """发送与 STM32 k230_proto.c 完全匹配的目标文本帧。"""
    frame = "X:%d,Y:%d,W:%d,H:%d\n" % (
        blob.cx(),
        blob.cy(),
        blob.w(),
        blob.h()
    )
    uart.write(frame)


def send_no_target():
    uart.write("NO_TARGET\n")


def draw_overlay(img, blob):
    """绘制画面中心和目标标记。"""
    center_x = IMAGE_WIDTH // 2
    center_y = IMAGE_HEIGHT // 2

    # 画面中心：白色十字
    img.draw_cross(
        center_x,
        center_y,
        color=(255, 255, 255),
        size=18,
        thickness=2
    )

    if blob is None:
        return

    # 目标框与目标中心：绿色
    img.draw_rectangle(
        blob.rect(),
        color=(0, 255, 0),
        thickness=3
    )
    img.draw_cross(
        blob.cx(),
        blob.cy(),
        color=(0, 255, 0),
        size=20,
        thickness=3
    )


def drain_stm32_messages():
    """可选：读取 STM32 发回的状态，避免接收缓存长期堆积。"""
    data = uart.read()
    if data:
        try:
            print("STM32:", data.decode().strip())
        except Exception:
            print("STM32 raw:", data)


def main():
    last_send_ms = time.ticks_ms()

    init_uart()
    init_camera_and_display()

    time.sleep_ms(300)
    uart.write("K230_READY\n")

    print("K230 green tracking started")
    print("vflip =", ENABLE_VERTICAL_FLIP)
    print("image center =", IMAGE_WIDTH // 2, IMAGE_HEIGHT // 2)

    while True:
        os.exitpoint()

        img = sensor.snapshot()

        blobs = img.find_blobs(
            GREEN_THRESHOLDS,
            pixels_threshold=PIXELS_THRESHOLD,
            area_threshold=AREA_THRESHOLD,
            merge=True,
            margin=MERGE_MARGIN
        )

        target = select_largest_blob(blobs)
        draw_overlay(img, target)

        now_ms = time.ticks_ms()
        can_send = (
            SEND_INTERVAL_MS == 0 or
            time.ticks_diff(now_ms, last_send_ms) >= SEND_INTERVAL_MS
        )

        if can_send:
            if target is not None:
                send_target(target)
            else:
                send_no_target()
            last_send_ms = now_ms

        Display.show_image(img)
        drain_stm32_messages()

        gc.collect()


try:
    main()

except KeyboardInterrupt:
    print("User stopped")

except BaseException as exc:
    print("Fatal error:", exc)
    raise

finally:
    try:
        if uart is not None:
            uart.write("NO_TARGET\n")
    except Exception:
        pass

    try:
        if sensor is not None:
            sensor.stop()
    except Exception:
        pass

    try:
        Display.deinit()
    except Exception:
        pass

    try:
        os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
        time.sleep_ms(100)
    except Exception:
        pass

    try:
        MediaManager.deinit()
    except Exception:
        pass

    try:
        if uart is not None:
            uart.deinit()
    except Exception:
        pass

    gc.collect()
