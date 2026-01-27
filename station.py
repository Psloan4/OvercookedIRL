import time
from enum import Enum, auto

from aruco_tag_detector import ArucoTagDetector
from feed_relay import FeedRelay


class StationState(Enum):
    WAITING = auto()
    SCANNING = auto()
    COMPLETE = auto()


class Station:
    COMPLETE_HOLD_SECONDS = 2.0

    def __init__(self, x, y, w, h, feed_relay: FeedRelay, scan_time: float):
        self.x = int(x)
        self.y = int(y)
        self.w = int(w)
        self.h = int(h)

        self.feed_relay = feed_relay
        self.tag_det = ArucoTagDetector()
        self.scan_time = float(scan_time)

        self.state: StationState = StationState.WAITING
        self.next_event_t: float | None = None  # absolute monotonic time

    # -------- core loop --------

    def tick(self) -> StationState:
        """
        Call once per tick AFTER feed_relay.update_image().
        """
        now = time.monotonic()

        if self.state == StationState.WAITING:
            return self._tick_waiting(now)

        if self.state == StationState.SCANNING:
            return self._tick_scanning(now)

        if self.state == StationState.COMPLETE:
            return self._tick_complete(now)

        # safety fallback
        self.set_state(StationState.WAITING)
        return self.state

    # -------- state handlers --------

    def _tick_waiting(self, now: float) -> StationState:
        if self._tag_present():
            # schedule scan completion
            self.set_state(StationState.SCANNING, next_event_t=now + self.scan_time)
        return self.state

    def _tick_scanning(self, now: float) -> StationState:
        if not self._tag_present():
            # continuous requirement violated -> reset
            self.set_state(StationState.WAITING, next_event_t=None)
            return self.state

        if self.next_event_t is None:
            # inconsistent state; recover by restarting scan window
            self.set_state(StationState.SCANNING, next_event_t=now + self.scan_time)
            return self.state

        if now >= self.next_event_t:
            # schedule end of COMPLETE hold
            self.set_state(StationState.COMPLETE, next_event_t=now + self.COMPLETE_HOLD_SECONDS)

        return self.state

    def _tick_complete(self, now: float) -> StationState:
        if self.next_event_t is None:
            # inconsistent state; recover by starting hold window
            self.set_state(StationState.COMPLETE, next_event_t=now + self.COMPLETE_HOLD_SECONDS)
            return self.state

        if now >= self.next_event_t:
            self.set_state(StationState.WAITING, next_event_t=None)

        return self.state

    # -------- single mutation point --------

    def set_state(self, state: StationState, next_event_t: float | None = None) -> None:
        """
        The only method that mutates state + timing.

        next_event_t is an absolute time.monotonic() timestamp for the next transition,
        or None if no scheduled transition.
        """
        self.state = state
        self.next_event_t = next_event_t

    # -------- helpers --------

    def _tag_present(self) -> bool:
        roi = self.feed_relay.get_sub_section(self.x, self.y, self.w, self.h)
        return self.tag_det.detect_first_id(roi) is not None

    def get_scan_progress(self) -> float:
        """
        Optional helper for UI progress bars.
        - WAITING: 0.0
        - SCANNING: 0.0..1.0
        - COMPLETE: 1.0
        """
        if self.state == StationState.WAITING:
            return 0.0
        if self.state == StationState.COMPLETE:
            return 1.0
        if self.state != StationState.SCANNING or self.next_event_t is None:
            return 0.0

        now = time.monotonic()
        scan_start = self.next_event_t - self.scan_time
        if now <= scan_start:
            return 0.0
        if now >= self.next_event_t:
            return 1.0
        return (now - scan_start) / self.scan_time
