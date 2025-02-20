import cv2
import numpy as np

class ColorDetectionAndTracking:
    def __init__(self):
        # Renk sınırları (HSV formatında)
        self.red_lower1 = np.array([0, 120, 70])  # Kırmızı ton 1
        self.red_upper1 = np.array([10, 255, 255])
        self.red_lower2 = np.array([170, 120, 70])  # Kırmızı ton 2
        self.red_upper2 = np.array([180, 255, 255])
        self.blue_lower = np.array([100, 150, 70])  # Mavi ton
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
                M = cv2.moments(largest_contour)
                if M["m00"] != 0:
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])
                    red_target = (cX, cY)
                    cv2.circle(frame, (cX, cY), 5, (0, 0, 255), -1)
                    print(f"Kırmızı Balon Tespit Edildi: ({cX}, {cY})")

        # Mavi balon için kontur bulma
        contours_blue, _ = cv2.findContours(blue_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if contours_blue:
            largest_contour = max(contours_blue, key=cv2.contourArea)
            if cv2.contourArea(largest_contour) > 50:  # Minimum alan
                M = cv2.moments(largest_contour)
                if M["m00"] != 0:
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])
                    cv2.circle(frame, (cX, cY), 5, (255, 0, 0), -1)
                    print(f"Mavi Balon Tespit Edildi: ({cX}, {cY})")

        return frame, red_target

# Kameradan görüntü yakalama
cap = cv2.VideoCapture(0)
detector = ColorDetectionAndTracking()

while True:
    ret, frame = cap.read()
    if not ret:
        break

    processed_frame, red_target = detector.detect_and_track(frame)
    
    cv2.imshow("Color Detection", processed_frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
