import cv2
import numpy as np
from motor_deneme import MotorControl
import threading
import queue


# Pin tanımlamaları
dir_pin_x = 17
step_pin_x = 18
dir_pin_y = 22
step_pin_y = 23
fire_pin = 24
emergency_pin = 25


# Motor kontrol nesnesi
motor = MotorControl(dir_pin_x, step_pin_x, dir_pin_y, step_pin_y, fire_pin, emergency_pin)

# Hedef pozisyonları tutmak için bir kuyruk oluştur
target_queue = queue.Queue()
current_pos_lock = threading.Lock()

# Kamera başlat
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
cap.set(cv2.CAP_PROP_FPS, 30)

if not cap.isOpened():
    print("Kamera başlatılamadı! Lütfen kamerayı kontrol edin.")
    exit()

 #Başlangıç pozisyonu
ret, frame = cap.read()
if ret:
    height, width, _ = frame.shape
    current_pos = [width // 2, height // 2]
    centerX, centerY = width // 2, height // 2
else:
    current_pos = [0, 0]
    centerX, centerY = 0, 0

# Görüntü işleme thread'i
def detection_and_tracking_thread(cap, display_callback, x_label, y_label, running_flag):
    kernel = np.ones((5, 5), np.uint8)

    while running_flag():
        ret, frame = cap.read()
        if not ret:
            break
        
        frame = cv2.flip(frame, 1)
        blur = cv2.medianBlur(frame, 5)
        hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)

        # Kırmızı için HSV aralığı
        lower_red_1 = np.array([0, 120, 120])
        upper_red_1 = np.array([10, 255, 255])
        lower_red_2 = np.array([170, 120, 120])
        upper_red_2 = np.array([180, 255, 255])

        # Mavi için HSV aralığı
        lower_blue = np.array([100, 150, 50])
        upper_blue = np.array([140, 255, 255])

        # Maskeleri oluştur
        mask_red = cv2.bitwise_or(cv2.inRange(hsv, lower_red_1, upper_red_1),
                                  cv2.inRange(hsv, lower_red_2, upper_red_2))
        mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)

        # Gürültü azaltma
        mask_red = cv2.erode(mask_red, kernel, iterations=1)
        mask_red = cv2.dilate(mask_red, kernel, iterations=2)
        mask_blue = cv2.erode(mask_blue, kernel, iterations=1)
        mask_blue = cv2.dilate(mask_blue, kernel, iterations=2)

        # Kırmızı balon tespiti
        contours_red, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours_red:
            max_contour = max(contours_red, key=cv2.contourArea)
            M = cv2.moments(max_contour)
            if M['m00'] != 0:
                cX_red = int(M['m10'] / M['m00'])
                cY_red = int(M['m01'] / M['m00'])
                target_pos = [cX_red, cY_red]


                if target_queue.qsize() < 3:
                    target_queue.put(target_pos)

                # Dinamik eşik ve ateşleme kontrolü
                area = cv2.contourArea(max_contour)
                dynamic_threshold = max(20, min(100, int(1000 / (area + 1))))
                if abs(cX_red - centerX) < dynamic_threshold and abs(cY_red - centerY) < dynamic_threshold:
                    motor.fire()

                # Kırmızı balonu çiz
                rect = cv2.minAreaRect(max_contour)
                box = cv2.boxPoints(rect)
                box = np.int64(box)
                cv2.drawContours(frame, [box], 0, (0, 0, 255), 2)
                cv2.circle(frame, (cX_red, cY_red), 5, (0, 0, 255), -1)
                cv2.putText(frame, "Dusman", (cX_red - 20, cY_red - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        # Mavi balon tespiti
        contours_blue, _ = cv2.findContours(mask_blue, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours_blue:
            max_contour = max(contours_blue, key=cv2.contourArea)
            M = cv2.moments(max_contour)
            if M['m00'] != 0:
                cX_blue = int(M['m10'] / M['m00'])
                cY_blue = int(M['m01'] / M['m00'])

                # Mavi balonu çiz
                rect = cv2.minAreaRect(max_contour)
                box = cv2.boxPoints(rect)
                box = np.int64(box)
                cv2.drawContours(frame, [box], 0, (255, 0, 0), 2)
                cv2.circle(frame, (cX_blue, cY_blue), 5, (255, 0, 0), -1)
                cv2.putText(frame, "Dost", (cX_blue - 20, cY_blue - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)

        # Görüntüyü göster
        display_callback(frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            target_queue.put(None)  # Motor thread'ini sonlandır
            break

cap.release()
cv2.destroyAllWindows()





