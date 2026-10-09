import queue
import cv2
import threading

# ─────────────────────────────────────────────────────────────────────────────
# Thread-safe video writer using a dedicated writer thread + queue
# ─────────────────────────────────────────────────────────────────────────────
class VideoRecorder:
    def __init__(self, path, fourcc, fps, size, is_color=True):
        self.writer = cv2.VideoWriter(path, cv2.VideoWriter_fourcc(*fourcc), fps, size, isColor=is_color)
        self.q = queue.Queue(maxsize=120)  # buffer up to 2 seconds at 60fps
        self.thread = threading.Thread(target=self._worker, daemon=True)
        self.thread.start()

    def _worker(self):
        while True:
            frame = self.q.get()
            if frame is None:  # poison pill → stop
                break
            self.writer.write(frame)
            self.q.task_done()

    def write(self, frame):
        try:
            self.q.put_nowait(frame.copy())  # .copy() avoids mutation races
        except queue.Full:
            pass  # drop frame if buffer is full rather than blocking

    def release(self):
        self.q.put(None)       # send poison pill
        self.thread.join()
        self.writer.release()