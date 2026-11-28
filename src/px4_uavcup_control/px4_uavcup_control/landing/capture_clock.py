"""Check simulated capture age without changing the controller's timer clock."""


def stamp_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


class CaptureClock:
    def __init__(self):
        self.latest_ns = None
        self.progress_at = float('-inf')
        self.reset = False

    def update(self, clock_ns, now):
        if self.latest_ns is not None and clock_ns < self.latest_ns:
            # A reset can mix frames from two runs. Restart the consumers.
            self.reset = True
        if self.latest_ns is None or clock_ns != self.latest_ns:
            self.latest_ns = clock_ns
            self.progress_at = now

    def is_fresh(self, capture_ns, now, limit_s):
        return (
            not self.reset
            and capture_ns is not None
            and self.latest_ns is not None
            and 0 <= now - self.progress_at <= limit_s
            and 0 <= self.latest_ns - capture_ns <= limit_s * 1e9
        )
