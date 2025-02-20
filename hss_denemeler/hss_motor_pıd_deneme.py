import Jetson.GPIO as GPIO
import time
import threading
import cv2

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

        # Acil durum butonu kontrolü için bir thread başlat
        threading.Thread(target=self.emergency_stop_listener, daemon=True).start()

    def move_motor(self, dir_pin, step_pin, steps, direction, control):
        GPIO.output(dir_pin, direction)
        min_delay = 0.0005  # En hızlı adım süresi (0.5 ms)
        max_delay = 0.005   # En yavaş adım süresi (5 ms)

        step_delay = max(min_delay, min(max_delay, 1 / (abs(control) + 1)))

        for _ in range(steps):
            if not self.running:
                break
            GPIO.output(step_pin, GPIO.HIGH)
            time.sleep(step_delay)
            GPIO.output(step_pin, GPIO.LOW)
            time.sleep(step_delay)

        print(f"Motor {steps} adım hareket etti, yön: {'HIGH' if direction == GPIO.HIGH else 'LOW'}, adım süresi: {step_delay:.4f} saniye")
        
    def move_to_target(self, current_pos, target_pos):
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
            self.move_motor(self.dir_pin_x, self.step_pin_x, steps_x, direction_x,control_x)
            # Mevcut X pozisyonunu güncelle
            current_pos[0] += steps_x if direction_x == GPIO.HIGH else -steps_x

        if steps_y > 0:
            self.move_motor(self.dir_pin_y, self.step_pin_y, steps_y, direction_y,control_y)
            # Mevcut Y pozisyonunu güncelle
            current_pos[1] += steps_y if direction_y == GPIO.HIGH else -steps_y

        print(f"Güncellenmiş pozisyon: X={current_pos[0]}, Y={current_pos[1]}")

    def fire(self):
        print("Ateşleme yapılıyor!")
        GPIO.output(self.fire_pin, GPIO.HIGH)
        time.sleep(0.5)
        GPIO.output(self.fire_pin, GPIO.LOW)
        print("Ateşleme tamamlandı.")

    def stop_all(self, cap):
        self.stop_motors()
        self.cleanup(cap)
        print("Motorlar ve kamera durduruldu.")

    def reset_system(self):
        self.running = True
        print("Sistem yeniden başlatıldı. Motorlar ve kamera hazır.")

    def emergency_stop_listener(self):
        while self.running:
            if GPIO.input(self.emergency_pin) == GPIO.LOW:
                self.stop_all(None)
                print("Acil durum butonuna basıldı! Motorlar ve kamera durduruldu.")
                self.running = False
                while GPIO.input(self.emergency_pin) == GPIO.LOW:
                    time.sleep(0.1)
                self.reset_system()

    def stop_motors(self):
        GPIO.output(self.step_pin_x, GPIO.LOW)
        GPIO.output(self.step_pin_y, GPIO.LOW)
        print("Motorlar durduruldu.")

    def cleanup(self, cap=None):
        try:
            GPIO.cleanup()
            print("GPIO pinleri temizlendi.")
        except RuntimeError:
            print("GPIO zaten temizlenmiş.")
        if cap:
            cap.release()
        cv2.destroyAllWindows()
        print("Motorlar durduruldu ve kaynaklar temizlendi.")
