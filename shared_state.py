import threading

class SharedState:
    def __init__(self):
        self._lock = threading.Lock()
        self._state = {}

    def set_state(self, new_state):
        with self._lock:
            self._state = new_state

    def get_state(self):
        with self._lock:
            return self._state
