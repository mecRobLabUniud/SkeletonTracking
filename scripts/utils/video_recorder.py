import queue
import cv2
import threading


# ─────────────────────────────────────────────────────────────────────────────
# Thread-safe video writer using a dedicated writer thread and a queue
# ─────────────────────────────────────────────────────────────────────────────
class VideoRecorder:
    # ─────────────────────────────────────────────────────────────────────────
    # Open the video file and start the background writer thread
    # ─────────────────────────────────────────────────────────────────────────
    def __init__(self, path, fourcc, fps, size, is_color=True):
        self.writer = cv2.VideoWriter(path, cv2.VideoWriter_fourcc(*fourcc), fps, size, isColor=is_color)
        self.q = queue.Queue(maxsize=120)
        self.thread = threading.Thread(target=self._worker, daemon=True)
        self.thread.start()

    # ─────────────────────────────────────────────────────────────────────────
    # Write frames from the queue until a None sentinel is received
    # ─────────────────────────────────────────────────────────────────────────
    def _worker(self):
        while True:
            frame = self.q.get()
            if frame is None:
                break
            self.writer.write(frame)
            self.q.task_done()

    # ─────────────────────────────────────────────────────────────────────────
    # Enqueue a copy of the frame, dropping it when the buffer is full
    # ─────────────────────────────────────────────────────────────────────────
    def write(self, frame):
        try:
            self.q.put_nowait(frame.copy())
        except queue.Full:
            pass

    # ─────────────────────────────────────────────────────────────────────────
    # Stop the writer thread and release the video file
    # ─────────────────────────────────────────────────────────────────────────
    def release(self):
        self.q.put(None)
        self.thread.join()
        self.writer.release()