import sensor
import time
from pyb import UART

# === Настройки ===
GRAYSCALE_THRESHOLD = [(0, 80)]

ROIS = [
    (0, 100, 160, 20, 0.7),
    (0, 80, 160, 20, 0.6),
    (0, 60, 160, 20, 0.5),
    (0, 40, 160, 20, 0.4),
    (0, 20, 160, 20, 0.3),
    (0, 0, 160, 20, 0.1),
]
weight_sum = sum(r[4] for r in ROIS)

# --- PID-параметры (подбираются экспериментально) ---
Kp = 2.0
Ki = 0.0
Kd = 0.3

# --- PID-переменные ---
previous_error = 0
integral = 0

# --- UART ---
uart = UART(3, 115200)  # TX=P4, RX=P5

# --- Инициализация камеры ---
sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE)
sensor.set_framesize(sensor.QQVGA)
sensor.skip_frames(time=2000)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)
clock = time.clock()

# Центр кадра (для QQVGA 160x120)
setpoint = 80

# --- Основной цикл ---
while True:
    clock.tick()
    img = sensor.snapshot()

    # Бинаризация и фильтрация
    img.binary(GRAYSCALE_THRESHOLD)
    img.erode(1)
    img.dilate(1)

    centroid_sum = 0
    weight_sum = 0
    line_found = False

    # Поиск линии по ROI
    for r in ROIS:
        img.draw_rectangle(r[0:4], color=127)
        blobs = img.find_blobs([(255, 255)], roi=r[0:4], merge=True)

        if blobs:
            line_found = True
            largest_blob = max(blobs, key=lambda b: b.pixels())
            cx = largest_blob.cx()
            cy = largest_blob.cy()
            centroid_sum += cx * r[4]
            weight_sum += r[4]
            img.draw_rectangle(largest_blob.rect(), color=255)
            img.draw_cross(cx, cy, color=255, size=5)

    # --- PID обработка ---
    if line_found:
        center_pos = centroid_sum / weight_sum
        error = center_pos - setpoint

        # --- PID ---
        integral += error
        integral = max(min(integral, 100), -100)  # анти-windup
        derivative = error - previous_error
        output = (Kp * error) + (Ki * integral) + (Kd * derivative)
        previous_error = error

        # --- Передача по UART ---
        error = center_pos - setpoint

        # --- Передача по UART ---
        # Отправляем только ошибку (e)
        uart.write("error:{:.2f}".format(error))


        # Отладочный вывод OpenMV
        # print("FPS: {:.2f} | Error: {:.2f} ".format(clock.fps(), error))
        print("error:{:.2f}".format(error))

    else:
        # --- Передача "nan" при потере линии ---
        uart.write("nan\n")
        # print("Line Lost")
