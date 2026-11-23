"""Move the opened carton into reach before folding and releasing it."""

from staged_compress import StagedCompressAndTip
from inside_shift import InsideShift


class TranslateAndFlatten:
    def __init__(self, env, previous):
        self.env = env
        self.shift = InsideShift(env, previous)
        self.controller = self.shift
        self.done = False
        self.failure = None

    def command(self, time):
        action, report = self.controller.command(time)
        self.failure = self.controller.failure
        if self.controller.done and self.failure is None:
            if self.controller is self.shift:
                self.controller = StagedCompressAndTip(self.env, action)
            else:
                self.done = True
        return action, report
