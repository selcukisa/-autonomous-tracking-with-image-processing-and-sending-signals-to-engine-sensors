import time

try:
    import RPi.GPIO as GPIO  # Raspberry Pi GPIO kütüphanesi
except (ImportError, RuntimeError):
    # GPIO kütüphanesi yoksa Mock GPIO oluştur
    class MockGPIO:
        BCM = 'BCM'
        OUT = 'OUT'
        HIGH = 'HIGH'
        LOW = 'LOW'

        @staticmethod
        def setmode(mode):
            print(f"GPIO setmode({mode})")

        @staticmethod
        def setup(pin, mode):
            print(f"GPIO setup(pin={pin}, mode={mode})")

        @staticmethod
        def output(pin, state):
            print(f"GPIO output(pin={pin}, state={state})")

        @staticmethod
        def cleanup():
            print("GPIO cleanup()")

    GPIO = MockGPIO()  # Mock GPIO atanır

class PIDController:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.previous_error = 0
        self.integral = 0

    def compute(self, setpoint, current_value):
        # Hata hesaplama
        error = setpoint - current_value
        proportional = self.Kp * error
        self.integral += error
        integral = self.Ki * self.integral
        derivative = self.Kd * (error - self.previous_error)
        self.previous_error = error

        # PID çıkışı
        return proportional + integral + derivative

class MotorControl:
    def __init__(self, dir_pin_x, step_pin_x, dir_pin_y, step_pin_y):
        # GPIO ayarları
        self.dir_pin_x = dir_pin_x
        self.step_pin_x = step_pin_x
        self.dir_pin_y = dir_pin_y
        self.step_pin_y = step_pin_y

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.dir_pin_x, GPIO.OUT)
        GPIO.setup(self.step_pin_x, GPIO.OUT)
        GPIO.setup(self.dir_pin_y, GPIO.OUT)
        GPIO.setup(self.step_pin_y, GPIO.OUT)

        # PID kontrol mekanizmaları
        self.pid_x = PIDController(1.0, 0.1, 0.05)  # X ekseni PID
        self.pid_y = PIDController(1.0, 0.1, 0.05)  # Y ekseni PID

        # Motor hareket sınırları
        self.max_speed = 1000  # Maksimum adım hızı
        self.min_speed = 100   # Minimum adım hızı

    def move_motor(self, dir_pin, step_pin, steps, direction):
        GPIO.output(dir_pin, direction)
        for _ in range(steps):
            GPIO.output(step_pin, GPIO.HIGH)
            time.sleep(0.01)  # Adım süresi
            GPIO.output(step_pin, GPIO.LOW)
            time.sleep(0.01)

    def move_to_target(self, current_pos, target_pos):
        if target_pos is None:
            print("Hedef bulunamadı, motor durduruldu.")
            return

        # Hedef pozisyonu ve mevcut pozisyon arasındaki hata
        target_x, target_y = target_pos
        current_x, current_y = current_pos

        # PID çıkışlarını hesapla
        control_x = self.pid_x.compute(target_x, current_x)
        control_y = self.pid_y.compute(target_y, current_y)

        # Hız ve yön ayarı
        steps_x = min(self.max_speed, max(self.min_speed, abs(int(control_x))))
        steps_y = min(self.max_speed, max(self.min_speed, abs(int(control_y))))

        direction_x = GPIO.HIGH if control_x > 0 else GPIO.LOW
        direction_y = GPIO.HIGH if control_y > 0 else GPIO.LOW

        # X ve Y eksenlerinde motor hareketi
        if steps_x > 0:
            self.move_motor(self.dir_pin_x, self.step_pin_x, steps_x, direction_x)
        if steps_y > 0:
            self.move_motor(self.dir_pin_y, self.step_pin_y, steps_y, direction_y)

        # Mevcut pozisyonu güncelle (örnek, sensörle entegre edilebilir)
        current_pos[0] += steps_x if direction_x == GPIO.HIGH else -steps_x
        current_pos[1] += steps_y if direction_y == GPIO.HIGH else -steps_y

        print(f"Motor hareketi: X={steps_x}, Y={steps_y} (Direction X={direction_x}, Y={direction_y})")

    def stop_motors(self):
        GPIO.output(self.step_pin_x, GPIO.LOW)
        GPIO.output(self.step_pin_y, GPIO.LOW)
        print("Motorlar durduruldu.")

    def cleanup(self):
        GPIO.cleanup()
        print("GPIO temizlendi.")
