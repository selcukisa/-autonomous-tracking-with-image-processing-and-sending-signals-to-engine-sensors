import cv2
from d_motor import MotorControl
import threading

# Motor pin tanımlamaları
dir_pin_x = 17
step_pin_x = 18
dir_pin_y = 22
step_pin_y = 23
fire_pin = 24
emergency_pin = 25

# Motor kontrol nesnesi
motor = MotorControl(dir_pin_x, step_pin_x, dir_pin_y, step_pin_y, fire_pin, emergency_pin)

# Kamera başlatma
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("Kamera başlatılamadı! Lütfen kamerayı kontrol edin.")
    motor.cleanup()
    exit()

# Acil durum thread'ini başlat
emergency_thread = threading.Thread(target=motor.emergency_stop_listener, daemon=True)
emergency_thread.start()

# Motor hareketlerini kolaylaştırmak için yardımcı fonksiyon
def manual_move(axis, direction, steps=50, control=1.0):
    """Belirtilen eksende ve yönde motoru hareket ettirir."""
    if axis == "x":
        motor.move_motor(dir_pin_x, step_pin_x, steps, direction, control)
    elif axis == "y":
        motor.move_motor(dir_pin_y, step_pin_y, steps, direction, control)

# Temizlik işlemlerini yapan fonksiyon
def cleanup_system():
    """Motor ve kamera kaynaklarını temizler."""
    motor.cleanup(cap)
    print("Sistem temizlendi ve sonlandırıldı.")

# Ana döngü: Kamera görüntüsü ve manuel kontrol
print("WASD ile motor kontrolü, 'K' ile ateşleme, 'Q' ile çıkış.")

while True:
    ret, frame = cap.read()
    if not ret:
        print("Kameradan görüntü alınamadı!")
        break

    # Görüntü üzerine merkez çizgileri ekleme
    height, width, _ = frame.shape
    centerX, centerY = width // 2, height // 2
    cv2.line(frame, (0, centerY), (width, centerY), (0, 0, 0), 2)
    cv2.line(frame, (centerX, 0), (centerX, height), (0, 0, 0), 2)

    # Görüntüyü göster
    cv2.imshow("Kamera Görüntüsü", frame)

    # Klavye girişlerini kontrol et
    key = cv2.waitKey(1) & 0xFF
    if key == ord("w"):
        print("Yukarı hareket")
        manual_move(axis="y", direction=0)
    elif key == ord("s"):
        print("Aşağı hareket")
        manual_move(axis="y", direction=1)
    elif key == ord("a"):
        print("Sola hareket")
        manual_move(axis="x", direction=1)
    elif key == ord("d"):
        print("Sağa hareket")
        manual_move(axis="x", direction=0)
    elif key == ord("k"):
        print("Ateşleme yapılıyor")
        motor.fire()
    elif key == ord("q"):
        print("Program sonlandırıldı.")
        break

# Temizlik işlemleri
cleanup_system()
