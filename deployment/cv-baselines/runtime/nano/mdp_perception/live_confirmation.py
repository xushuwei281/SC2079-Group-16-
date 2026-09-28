"""Confirmation across distinct processed frames, without a detection cache."""


class LiveConfirmation:
    def __init__(self, required_frames=3, absence_seconds=2.0, max_gap_seconds=3.0):
        if required_frames < 1 or absence_seconds <= 0 or max_gap_seconds <= 0:
            raise ValueError("Confirmation settings must be positive")
        self.required_frames = required_frames
        self.absence_seconds = absence_seconds
        self.max_gap_seconds = max_gap_seconds
        self.reset()

    def reset(self):
        self.candidate = None
        self.count = 0
        self.last_published = None
        self.absent_since = None
        self.last_observation = None

    def break_sequence(self):
        """A missing/stale camera frame is not evidence that a target vanished."""
        self.candidate = None
        self.count = 0
        self.absent_since = None
        self.last_observation = None

    def observe(self, symbol_id, observed_at):
        if self.last_observation is not None:
            if observed_at <= self.last_observation:
                return None
            if observed_at - self.last_observation > self.max_gap_seconds:
                self.break_sequence()
        self.last_observation = observed_at
        if symbol_id is None:
            self.candidate = None
            self.count = 0
            if self.absent_since is None:
                self.absent_since = observed_at
            elif observed_at - self.absent_since >= self.absence_seconds:
                self.last_published = None
            return None
        if type(symbol_id) is not int or not 11 <= symbol_id <= 40:
            self.break_sequence()
            return None
        self.absent_since = None
        if symbol_id == self.candidate:
            self.count += 1
        else:
            self.candidate = symbol_id
            self.count = 1
        if self.count >= self.required_frames and symbol_id != self.last_published:
            self.last_published = symbol_id
            return symbol_id
        return None
