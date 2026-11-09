"""Small, fresh JPEG previews; no video or image data enters IoTDA telemetry."""
import threading
import time


class CameraBuffer:
    def __init__(self, clock=time.monotonic):
        self.clock = clock
        self.lock = threading.Lock()
        self.encoding = threading.Lock()
        self.next_frame = 0
        self.frame = None
        self.sequence = 0

    def update(self, image):
        if not self.encoding.acquire(blocking=False):
            return False
        try:
            now = self.clock()
            if now < self.next_frame:
                return False
            self.next_frame = now + 1
            import cv2
            height, width = image.shape[:2]
            if height <= 0 or width <= 0 or height * width > 9000000:
                raise ValueError('invalid preview size')
            scale = min(640 / width, 480 / height, 1)
            image = cv2.resize(image, (max(1, int(width * scale)), max(1, int(height * scale))))
            for _ in range(5):
                for quality in (55, 40, 28):
                    ok, encoded = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, quality])
                    if ok and len(encoded) <= 32000:
                        with self.lock:
                            self.sequence += 1
                            self.frame = (encoded.tobytes(), self.clock(), self.sequence)
                        return True
                height, width = image.shape[:2]
                image = cv2.resize(image, (max(1, width * 2 // 3), max(1, height * 2 // 3)))
            raise ValueError('preview cannot fit byte budget')
        finally:
            self.encoding.release()

    def get(self):
        with self.lock:
            if self.frame is None:
                return None
            jpeg, stamp, sequence = self.frame
            age = self.clock() - stamp
            return (jpeg, age, sequence) if 0 <= age <= 3 else None
