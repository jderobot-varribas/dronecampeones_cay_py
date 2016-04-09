# <Python port>
# Copyright (c) 2016
# Author: Victor Arribas <v.arribas.urjc@gmail.com>
# License: GPLv3 <http://www.gnu.org/licenses/gpl-3.0.html>

#  <PID C++>
# Copyright (c) 2015
# Author: Victor Arribas <v.arribas.urjc@gmail.com>
# Domain: Master Vision Artificial, URJC
# License: GPLv3 <http://www.gnu.org/licenses/gpl-3.0.html>


__author__ = 'varribas'


class PID:
    def __init__(self, reference=0, P=0, D=0, I=0, I_len=0):
        self.reference = reference

        self.P = P
        self.D = D
        self.I = I

        self.I_len = I_len
        self._error_history = []

        self.last_value = 0
        self.historical_error = 0
        self.alpha = 0.1

        self._feedback = 0

        self._errP = 0
        self._errD = 0
        self._errI = 0

    def setReference(self, reference):
        self.reference = reference

    def feedback(self, value=None):
        # idempotency
        if value is None:
            return self._feedback

        # error calculation
        self._errP = value - self.reference
        self._errD = self._errP - self.last_value
        if self.I_len > 0:
            self._error_history += [self._errP]
            n = len(self._error_history)
            if n > self.I_len:
                self._error_history = self._error_history[n-self.I_len:n]
            self._errI = sum(self._error_history)
        else:
            self._errI = self.alpha*self._errP + (1-self.alpha)*self.historical_error  # weighted mean

        # update
        self.last_value = value;
        self.historical_error = self._errI;

        # compute
        self._feedback = self.P * self._errP + self.D * self._errD + self.I * self._errI;
        return self._feedback;

    def weight(self):
        return self.P + self.D + self.I

    def reset(self):
        self.last_value = self.reference
        self.historical_error = 0
        self._error_history = []
        self._errP = 0
        self._errD = 0
        self._errI = 0
