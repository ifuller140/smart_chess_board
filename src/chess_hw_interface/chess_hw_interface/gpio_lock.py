#!/usr/bin/env python3
"""
Shared file-lock mutex protecting the CoreXY stepper motor GPIO pins from
concurrent access by more than one independent pigpio client.

Production nodes (stepper_driver_node, homing_node) each hold a SHARED lock
for their entire lifetime — they already coordinate step timing with each
other through ROS topics/services, so they're safe to run concurrently.

Bare-metal hardware tests that open their own pigpio connection directly to
the same pins (test_raw_motor, test_timing_sweep) must acquire an EXCLUSIVE
lock before touching the motors. That acquisition fails immediately
(non-blocking) if a production node currently holds the shared lock,
preventing two independent pigpio clients from racing step pulses on the
same physical pins.
"""
import fcntl
import os

LOCK_PATH = '/tmp/.chess_gantry_pins.lock'


class GantryPinLock:
    """Non-blocking flock()-based mutex handle. One instance per holder."""

    def __init__(self, path: str = LOCK_PATH):
        self._path = path
        self._fd = None

    def _open(self):
        if self._fd is None:
            self._fd = os.open(self._path, os.O_CREAT | os.O_RDWR, 0o666)

    def acquire_shared(self) -> bool:
        """Non-blocking. Compatible with other shared holders."""
        self._open()
        try:
            fcntl.flock(self._fd, fcntl.LOCK_SH | fcntl.LOCK_NB)
            return True
        except OSError:
            self.release()
            return False

    def acquire_exclusive(self) -> bool:
        """Non-blocking. Fails if any shared or exclusive holder exists."""
        self._open()
        try:
            fcntl.flock(self._fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            return True
        except OSError:
            self.release()
            return False

    def release(self):
        if self._fd is not None:
            try:
                fcntl.flock(self._fd, fcntl.LOCK_UN)
            except OSError:
                pass
            os.close(self._fd)
            self._fd = None
