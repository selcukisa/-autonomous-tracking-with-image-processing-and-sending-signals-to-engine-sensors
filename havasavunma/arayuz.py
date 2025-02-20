import sys
import subprocess
import threading
from PyQt5.QtWidgets import QApplication, QMainWindow
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtCore import QTimer
from arayuz_ui import Ui_MainWindow

class MainWindow(QMainWindow, Ui_MainWindow):
    def __init__(self):
        super().__init__()
        self.setupUi(self)

        # Başlatılan süreci takip etmek için
        self.process = None
        self.current_mode = None
        
        # Kamera güncellemesi için timer
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_camera_feed)

        # Butonlara işlev bağlama
        self.start.clicked.connect(self.start_system)
        self.stop.clicked.connect(self.stop_system)
        self.acil_durum.clicked.connect(self.emergency_stop)
        self.modsec.currentIndexChanged.connect(self.change_mode)

        # Başlangıçta motor durumu OFF
        self.motor_d_cikti.setText("OFF")

        # Başlangıçta koordinatlar "-"
        self.x_dur.setText("-")
        self.y_dur.setText("-")

    def start_system(self):
        if self.current_mode == "MANUEL MOD":
            self.process = subprocess.Popen(["python", "manuel_main_hss.py"])
            self.motor_d_cikti.setText("ON")
            self.x_dur.setText("-")
            self.y_dur.setText("-")
        elif self.current_mode == "OTONOM MOD":
            self.process = subprocess.Popen(["python", "hss_main.py"], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
            self.motor_d_cikti.setText("ON")
            self.timer.start(30)
            threading.Thread(target=self.read_coordinates, daemon=True).start()
        else:
            print("Lütfen bir mod seçin!")

    def stop_system(self):
        if self.process:
            self.process.terminate()
            self.process = None
            self.motor_d_cikti.setText("OFF")
            self.x_dur.setText("-")
            self.y_dur.setText("-")
            self.timer.stop()
        else:
            print("Çalışan bir sistem yok!")

    def emergency_stop(self):
        self.stop_system()
        self.motor_d_cikti.setText("OFF")
        print("Acil durum: Sistem kapatıldı!")

    def change_mode(self):
        mode = self.modsec.currentText()
        self.current_mode = mode
        self.stop_system()
        print(f"{mode} seçildi.")

    def read_coordinates(self):
        """Otonom modda çalışan süreçten koordinatları okur ve ekranda gösterir."""
        for line in self.process.stdout:
            if "cX_red" in line and "cY_red" in line:
                try:
                    parts = line.strip().split(",")
                    cX_red = parts[0].split(":")[1].strip()
                    cY_red = parts[1].split(":")[1].strip()
                    self.x_dur.setText(cX_red)
                    self.y_dur.setText(cY_red)
                except IndexError:
                    continue

    def update_camera_feed(self):
        """Gelen görüntüyü QLabel üzerinde gösterir."""
        data = self.process.stdout.read(1024 * 1024)
        if data:
            image = QImage.fromData(data)
            if not image.isNull():
                self.camera.setPixmap(QPixmap.fromImage(image))
                self.camera.setScaledContents(True)

    def closeEvent(self, event):
        self.stop_system()
        event.accept()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())
