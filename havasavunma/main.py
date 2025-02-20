import cv2
from motor_control import MotorControl
from utils import draw_detections, log  # utils dosyasındaki gerekli fonksiyonlar
import threading
import numpy as np

class ColorDetectionAndTracking:
    def __init__(self):
        # Kırmızı renk sınırları (HSV formatında)
        self.red_lower1 = np.array([0, 120, 70])  # Kırmızı ton 1
        self.red_upper1 = np.array([10, 255, 255])
        self.red_lower2 = np.array([170, 120, 70])  # Kırmızı ton 2
        self.red_upper2 = np.array([180, 255, 255])

        # Mavi renk sınırları (HSV formatında)
        self.blue_lower = np.array([100, 150, 70])
        self.blue_upper = np.array([140, 255, 255])

    def detect_and_track(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)  # HSV renk uzayına dönüştür
        red_target = None  # Kırmızı balonun hedef pozisyonu

        # Kırmızı maske oluştur
        red_mask1 = cv2.inRange(hsv, self.red_lower1, self.red_upper1)
        red_mask2 = cv2.inRange(hsv, self.red_lower2, self.red_upper2)
        red_mask = cv2.add(red_mask1, red_mask2)

        # Mavi maske oluştur
        blue_mask = cv2.inRange(hsv, self.blue_lower, self.blue_upper)

        # Gürültüyü azaltmak için bulanıklaştırma
        red_mask = cv2.medianBlur(red_mask, 5)
        blue_mask = cv2.medianBlur(blue_mask, 5)

        # Kırmızı balon için kontur bulma
        contours_red, _ = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if contours_red:
            largest_contour = max(contours_red, key=cv2.contourArea)
            if cv2.contourArea(largest_contour) > 50:  # Minimum alan
                x, y, w, h = cv2.boundingRect(largest_contour)
                red_target = (x + w // 2, y + h // 2)
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
                cv2.putText(frame, "Düşman", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                log(f"Kırmızı Balon Tespit Edildi: ({red_target[0]}, {red_target[1]})")

        # Mavi balon için kontur bulma
        contours_blue, _ = cv2.findContours(blue_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if contours_blue:
            largest_contour = max(contours_blue, key=cv2.contourArea)
            if cv2.contourArea(largest_contour) > 50:  # Minimum alan
                x, y, w, h = cv2.boundingRect(largest_contour)
                cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
                cv2.putText(frame, "Dost", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
                log(f"Mavi Balon Tespit Edildi: ({x + w // 2}, {y + h // 2})")

        return frame, red_target

# Modülleri başlat
log("Sistem başlatılıyor...")
detector_tracker = ColorDetectionAndTracking()
motor_controller = MotorControl(dir_pin_x=20, step_pin_x=21, dir_pin_y=22, step_pin_y=23)

# Kamerayı başlat
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)  # Çözünürlüğü küçült
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
cap.set(cv2.CAP_PROP_FPS, 15)  # FPS'yi sınırla

# Hedef pozisyon değişkenleri
red_target = None
current_position = None
running = True  # Thread'i kontrol etmek için koşul

# Motor kontrol işlemini ayrı bir iş parçacığında çalıştır
def motor_control_loop():
    global red_target, current_position, running
    while running:
        if red_target and current_position:
            motor_controller.move_to_target(current_position, red_target)
        else:
            motor_controller.stop_motors()

# Motor kontrol thread'i başlat
motor_thread = threading.Thread(target=motor_control_loop, daemon=True)
motor_thread.start()

try:
    # Başlangıç pozisyonu (örneğin kameranın ortası)
    frame_count = 0

    while True:
        # Kameradan görüntü al
        ret, frame = cap.read()
        if not ret:
            log("Kamera görüntüsü alınamadı.", level="ERROR")
            break

        # Görüntüyü yatay olarak çevir
        frame = cv2.flip(frame, 1)

        frame_count += 1

        # Yalnızca belirli karelerde tespit yap
        if frame_count % 5 == 0:  # Her 5 karede bir tespit yap
            frame, red_target = detector_tracker.detect_and_track(frame)

        # Mevcut pozisyonu güncelle (örneğin, merkez pozisyonu takip ediyoruz)
        if current_position is None:
            current_position = [frame.shape[1] // 2, frame.shape[0] // 2]
            log(f"Mevcut pozisyon başlangıçta merkeze ayarlandı: {current_position}")

        # Görselleştirme
        frame = draw_detections(frame, red_target=red_target)

        # Görüntüyü göster
        cv2.imshow("Balloon Detection and Tracking", frame)

        # Çıkış için 'q' tuşuna bas
        if cv2.waitKey(1) & 0xFF == ord('q'):
            log("Sistem durduruluyor...")
            break

except KeyboardInterrupt:
    log("Kullanıcı tarafından durduruldu.", level="WARNING")

finally:
    # Thread'i durdur
    running = False
    motor_thread.join()  # Thread'in bitmesini bekle
    # Kaynakları temizle
    cap.release()
    motor_controller.cleanup()
    cv2.destroyAllWindows()
