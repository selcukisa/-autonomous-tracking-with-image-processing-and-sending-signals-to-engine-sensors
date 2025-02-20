import sys
import threading
import cv2
import gc
from PyQt5.QtWidgets import QApplication, QMainWindow
from PyQt5.QtGui import QImage, QPixmap
from arayuz_ui_deneme import Ui_MainWindow
from hss_main_deneme import detection_and_tracking_thread
from manuel_main_hss_deneme import manual_control  
from motor_deneme import MotorControl

def load_stylesheet(app, filename="styles.qss"):
    """Verilen QSS stil dosyasını yükler."""
    with open(filename, "r") as file:
        app.setStyleSheet(file.read())

class MainApp(QMainWindow, Ui_MainWindow):
    def __init__(self):
        super().__init__()
        self.setupUi(self)
        self.motor = MotorControl(17, 18, 22, 23, 24, 25)
        self.running = False
        self.current_mode = None
        self.cap = None
        self.thread = None
        
        # Buton bağlantıları
        self.start.clicked.connect(self.start_system)
        self.stop.clicked.connect(self.stop_system)
        #self.acil_durum.clicked.connect(self.emergency_stop)

    def start_system(self):
        

        if self.running:
            print("Sistem zaten çalışıyor.")
            return
        if self.cap:
            self.cap.release()

        self.running = True
        self.cap = cv2.VideoCapture(0)  


        if not self.cap.isOpened():
            print("Kamera başlatılamadı!")
            self.running = False
            return

        if self.otoman.value() == 0:  # Otonom Mod
            print("Otonom Mod Başlatıldı.")
            self.current_mode = "OTONOM"
            self.thread = threading.Thread(target=detection_and_tracking_thread, 
                                           args=(self.cap, self.display_frame, self.add_target_to_queue, None, lambda: self.running))
        else:  # Manuel Mod
            print("Manuel Mod Başlatıldı.")
            self.current_mode = "MANUEL"
            self.thread = threading.Thread(target=manual_control, 
                                           args=(self.display_frame, lambda: self.running))

        self.thread.start()

    def add_target_to_queue(self, target_pos):
        """Hedef pozisyonu motor kontrol kuyruğuna ekler."""
        self.motor.target_queue.put(target_pos)

    def stop_system(self):
        if not self.running:
            print("Sistem zaten durdurulmuş.")
            return
        
        print("Sistem Durduruluyor...")
        self.running = False
        
        # Thread'i durdurma
        if self.thread and self.thread.is_alive():
            self.thread.join()
            self.thread = None
            
        # Kamerayı serbest bırakma
        if self.cap:
            self.cap.release()
            self.cap = None

        # OpenCV pencerelerini kapatma
        cv2.destroyAllWindows()

        # Çöp toplayıcısını çağırma
        gc.collect()

        # QLabel'i temizleme
        if self.camera:
            self.camera.clear()

        print("Sistem durduruldu ve kamera serbest bırakıldı.")

    #def emergency_stop(self):
        #print("Acil Durum: Sistem Durduruldu!")
        #self.stop_system()

    def display_frame(self, frame):
        """OpenCV'den alınan görüntüyü QLabel'e aktarır."""
        if self.running:
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            h, w, ch = frame.shape
            q_img = QImage(frame.data, w, h, ch * w, QImage.Format_RGB888)
            self.camera.setPixmap(QPixmap.fromImage(q_img))
            self.camera.setScaledContents(True)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    load_stylesheet(app, "styles.qss")
    window = MainApp()
    window.show()
    sys.exit(app.exec_())
