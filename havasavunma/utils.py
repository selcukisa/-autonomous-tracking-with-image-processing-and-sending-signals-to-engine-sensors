import cv2
import numpy as np

def TryExcept(func):
    try:
        return func()
    except Exception as e:
        print(f"Error: {e}")
        return None


def draw_detections(frame, red_target=None, blue_targets=None):
    """
    Algılanan balonları çizer.
    
    Args:
        frame: Görüntü karesi.
        red_target: Kırmızı balonun merkezi koordinatları (x, y) veya None.
        blue_targets: Mavi balonların merkezi koordinatlarının listesi [(x1, y1), ...] veya None.
        
    Returns:
        Görselleştirilmiş kare.
    """
    # Kırmızı balonu işaretle
    if red_target:
        center_x, center_y = int(red_target[0]), int(red_target[1])
        cv2.circle(frame, (center_x, center_y), 10, (0, 0, 255), -1)  # Kırmızı nokta
        cv2.putText(frame, "Red Target", (center_x + 10, center_y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

    # Mavi balonları işaretle
    if blue_targets:
        for target in blue_targets:
            center_x, center_y = int(target[0]), int(target[1])
            cv2.circle(frame, (center_x, center_y), 10, (255, 0, 0), -1)  # Mavi nokta
            cv2.putText(frame, "Blue Target", (center_x + 10, center_y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)

    return frame

def normalize_coordinates(coords, frame_shape):
    """
    Koordinatları normalize eder.
    
    Args:
        coords: Koordinatlar (x1, y1, x2, y2).
        frame_shape: Görüntü boyutları (yükseklik, genişlik).
    
    Returns:
        Normalize edilmiş koordinatlar (x1_norm, y1_norm, x2_norm, y2_norm).
    """
    h, w = frame_shape[:2]
    x1, y1, x2, y2 = coords
    return x1 / w, y1 / h, x2 / w, y2 / h

def log(message, level="INFO"):
    """
    Basit loglama işlevi.
    
    Args:
        message: Log mesajı.
        level: Log seviyesi ("INFO", "WARNING", "ERROR").
    """
    print(f"[{level}]: {message}")

def apply_blur(mask, kernel_size=5):
    """
    Bir maskeye bulanıklaştırma uygular.
    
    Args:
        mask: Giriş maskesi (binary image).
        kernel_size: Median blur için kernel boyutu.
        
    Returns:
        Bulanıklaştırılmış maske.
    """
    return cv2.medianBlur(mask, kernel_size)

def find_contours(mask, min_area=50):
    """
    Maske üzerinde konturları bulur ve filtreler.
    
    Args:
        mask: Giriş maskesi (binary image).
        min_area: Konturların minimum alanı.
    
    Returns:
        Filtrelenmiş konturlar listesi.
    """
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    filtered_contours = [cnt for cnt in contours if cv2.contourArea(cnt) > min_area]
    return filtered_contours

def calculate_center(contour):
    """
    Bir konturun merkezini hesaplar.
    
    Args:
        contour: Kontur (cv2.findContours sonucu).
    
    Returns:
        Merkez koordinatları (x, y) veya None.
    """
    M = cv2.moments(contour)
    if M["m00"] != 0:
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
        return cX, cY
    return None
