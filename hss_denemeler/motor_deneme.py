import time
import threading
import queue
import cv2

# GPIO için mock oluştur (Jetson Nano kullanımı için)
try:
    import Jetson.GPIO as GPIO
except ImportError:
    from unittest import mock
    GPIO = mock.Mock()
    GPIO.HIGH = 1
    GPIO.LOW = 0
    GPIO.BCM = 'BCM'
    GPIO.OUT = 'OUT'
    GPIO.IN = 'IN'
    GPIO.PUD_UP = 'PUD_UP'

# PID Kontrolcü Sınıfı
class PIDController:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.previous_error = 0
        self.integral = 0

    def compute(self, setpoint, current_value):
        error = setpoint - current_value
        proportional = self.Kp * error
        self.integral += error
        integral = self.Ki * self.integral
        derivative = self.Kd * (error - self.previous_error)
        self.previous_error = error

        return proportional + integral + derivative

# Motor Kontrol Sınıfı
class MotorControl:
    def __init__(self, dir_pin_x, step_pin_x, dir_pin_y, step_pin_y, fire_pin, emergency_pin, pid_params_x=(1.0, 0.1, 0.05), pid_params_y=(1.0, 0.1, 0.05)):
        # Motor pinleri
        self.dir_pin_x = dir_pin_x
        self.step_pin_x = step_pin_x
        self.dir_pin_y = dir_pin_y
        self.step_pin_y = step_pin_y

        # Ateşleme ve acil durum pinleri
        self.fire_pin = fire_pin
        self.emergency_pin = emergency_pin

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.dir_pin_x, GPIO.OUT)
        GPIO.setup(self.step_pin_x, GPIO.OUT)
        GPIO.setup(self.dir_pin_y, GPIO.OUT)
        GPIO.setup(self.step_pin_y, GPIO.OUT)
        GPIO.setup(self.fire_pin, GPIO.OUT)
        GPIO.setup(self.emergency_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        # PID kontrolcüleri
        self.pid_x = PIDController(*pid_params_x)
        self.pid_y = PIDController(*pid_params_y)

        self.running = True

        # Motor kontrol thread'ini başlat
        self.target_queue = queue.Queue()
        self.current_pos = [0, 0]
        self.current_pos_lock = threading.Lock()
        self.motor_thread = threading.Thread(target=self.motor_control_thread, daemon=True)
        self.motor_thread.start()

        # Acil durum butonu kontrolü için bir thread başlat
        threading.Thread(target=self.emergency_stop_listener, daemon=True).start()

    def move_motor(self, dir_pin, step_pin, steps, direction, control):
        """Motoru belirtilen adım sayısı ve yönde hareket ettirir."""
        GPIO.output(dir_pin, direction)
        for _ in range(steps):
            GPIO.output(step_pin, GPIO.HIGH)
            time.sleep(0.001)
            GPIO.output(step_pin, GPIO.LOW)
            time.sleep(0.001)
        print(f"Motor {steps} adım hareket etti, yön: {'HIGH' if direction == GPIO.HIGH else 'LOW'}")

    def move_to_target(self, current_pos, target_pos):
        """Hedef pozisyona motorları hareket ettirir."""
        if target_pos is None:
            print("Hedef bulunamadı, motor durduruldu.")
            return

        target_x, target_y = target_pos
        current_x, current_y = current_pos

        # PID hesaplamaları
        control_x = self.pid_x.compute(target_x, current_x)
        control_y = self.pid_y.compute(target_y, current_y)

        steps_x = abs(int(control_x))
        steps_y = abs(int(control_y))

        direction_x = GPIO.HIGH if control_x > 0 else GPIO.LOW
        direction_y = GPIO.HIGH if control_y > 0 else GPIO.LOW

        # Motorları hareket ettir
        if steps_x > 0:
            self.move_motor(self.dir_pin_x, self.step_pin_x, steps_x, direction_x, control_x)
        if steps_y > 0:
            self.move_motor(self.dir_pin_y, self.step_pin_y, steps_y, direction_y, control_y)

    def fire(self):
        """Ateşleme mekanizmasını çalıştırır."""
        print("Ateşleme yapılıyor!")
        GPIO.output(self.fire_pin, GPIO.HIGH)
        time.sleep(0.5)
        GPIO.output(self.fire_pin, GPIO.LOW)
        print("Ateşleme tamamlandı.")

    def stop_all(self, cap):
        """Motorları ve kamerayı durdurur."""
        self.cleanup(cap)
        print("Motorlar ve kamera durduruldu.")
        print("Motorlar durduruldu.")

    def reset_system(self):
        """Sistemi yeniden başlatır."""
        self.running = True
        print("Sistem yeniden başlatıldı. Motorlar ve kamera hazır.")

    def emergency_stop_listener(self):
        """Acil durdurma butonunu dinler."""
        while self.running:
            if GPIO.input(self.emergency_pin) == GPIO.LOW:
                print("Acil durum: Motorlar durduruldu!")
                self.stop_motors()
                self.running = False
            time.sleep(0.1)
            
    def cleanup(self, cap=None):
        """GPIO pinlerini temizler ve kamerayı serbest bırakır."""
        GPIO.cleanup()
        if cap:
            cap.release()
        cv2.destroyAllWindows()
        print("GPIO pinleri ve kaynaklar temizlendi.")

    def motor_control_thread(self):
        """Motor hareketlerini kontrol eden thread."""
        while self.running:
            target_pos = self.target_queue.get()
            if target_pos is None:
                break
            with self.current_pos_lock:
                self.move_to_target(self.current_pos, target_pos)
                self.current_pos[0] = target_pos[0]
                self.current_pos[1] = target_pos[1]

# Test için MotorControl nesnesi oluştur
if __name__ == "__main__":
    motor = MotorControl(17, 18, 22, 23, 24, 25)

    # Örnek hedef pozisyonlar ekleyelim
    motor.target_queue.put([100, 100])
    motor.target_queue.put([200, 200])
    motor.target_queue.put([300, 300])

    motor.fire()
    motor.stop_all(None)
