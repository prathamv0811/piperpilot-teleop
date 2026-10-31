"""Bounded input mailbox; Portal callbacks never command a motor."""
from collections import deque
import threading
import time

from contract import decode_quest


class RemoteQuestReader:
    def __init__(self, allowed_operator, timeout_s=.25, clock=time.monotonic):
        self.allowed_operator = allowed_operator
        self.timeout_s = timeout_s
        self.clock = clock
        self._lock = threading.RLock()
        self._state = None
        self._operator = None
        self._sequence = -1
        self._epoch = None
        self._retired_epochs = set()
        self._observations = {}
        self._rates = deque(maxlen=100)
        self.generation = 0
        self.accepted = 0
        self.rejected = 0
        self.last_rejection = ''

    def set_operator(self, identity):
        with self._lock:
            if identity != self._operator:
                self._operator = identity
                self._state = None
                self._rates.clear()
                self.generation += 1

    def sent_observation(self, timestamp_us):
        with self._lock:
            now = self.clock()
            self._observations[timestamp_us] = now
            self._observations = {k: v for k, v in self._observations.items() if now-v <= self.timeout_s}

    def accept(self, action):
        with self._lock:
            now = self.clock()
            try:
                if action.sender != self.allowed_operator or action.sender != self._operator:
                    raise ValueError('sender does not hold authorized control')
                sent = self._observations.get(action.in_reply_to_ts_us)
                if sent is None or not 0 <= now-sent <= self.timeout_s:
                    raise ValueError('action does not reference a fresh robot observation')
                frame = decode_quest(action.values, received_mono=now, transport_age_s=now-sent)
                if now-frame.recv_mono > self.timeout_s:
                    raise ValueError('Quest plus round-trip age exceeds timeout')
                epoch = int(action.values['epoch'])
                if epoch in self._retired_epochs:
                    raise ValueError('retired operator session')
                if epoch != self._epoch:
                    if self._epoch is not None:
                        self._retired_epochs.add(self._epoch)
                    self._epoch = epoch
                    self._sequence = -1
                    self.generation += 1
                if frame.frame <= self._sequence:
                    raise ValueError('duplicate or reordered action')
                self._sequence = frame.frame
                self._state = frame
                self._rates.append(now)
                self.accepted += 1
                return True
            except (ValueError, TypeError, KeyError, AttributeError) as exc:
                self.rejected += 1
                self.last_rejection = str(exc)
                # Rejected traffic cannot refresh the last accepted command.
                return False

    def get(self):
        with self._lock:
            return self._state

    def age(self):
        with self._lock:
            return float('inf') if self._state is None else self.clock()-self._state.recv_mono

    def rate_hz(self):
        with self._lock:
            if len(self._rates) < 2 or self.clock()-self._rates[-1] > 1:
                return 0.
            return (len(self._rates)-1)/(self._rates[-1]-self._rates[0])

    def send_haptic(self, *args, **kwargs):
        # Haptics are local UI feedback and are not part of the motor contract.
        pass
