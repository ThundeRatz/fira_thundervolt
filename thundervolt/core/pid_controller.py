import logging


class pidController:

    def __init__(self, kp, ki, kd, set_point=0, freq=1, saturation=None, max_integral=10, integral_fade_rate=1, settling_value=1e-2):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.freq = freq
        self._set_point = set_point
        self.saturation = saturation
        self.max_integral = max_integral
        self.integral_fade_rate = integral_fade_rate
        self.settling_value = settling_value

        self.set_point_changed = True
        self.error_acc = 0  # accumulated error for i term
        self.prev_error = 0  # previous error for d term

    def __str__(self):
        return f"PID - Kp: {self.kp:.02f} Ki: {self.ki:.02f} Kd: {self.kd:.02f} SP: {self._set_point:.02f} Freq: {self.freq:.02f} Hz"

    def __repr__(self):
        return f"pidController(Kp: {self.kp:.02f} Ki: {self.ki:.02f} Kd: {self.kd:.02f} SP: {self._set_point:.02f} Freq: {self.freq:.02f} Hz)"

    @property
    def set_point(self):
        return self._set_point

    @set_point.setter
    def set_point(self, value):
        self._set_point = value
        self.set_point_changed = True

    def update(self, state):
        error = self._set_point - state

        # Prevents spikes when set point is changed
        if self.set_point_changed:
            self.prev_error = error
            self.set_point_changed = False

        dedt = (error - self.prev_error)*self.freq
        self.prev_error = error

        # Anti-windup system to decrease integrative instability
        if self.saturation is None or error*self.kp <= self.saturation:
            self.error_acc += error/self.freq
        else:
            self.error_acc *= self.integral_fade_rate**(1/self.freq)

        if abs(self.ki*self.error_acc) > self.max_integral:
            self.error_acc = self.max_integral * \
                self.error_acc/abs(self.error_acc)

        response = self.kp*(error + self.ki*self.error_acc + self.kd*dedt)

        if self.saturation is not None and response >= self.saturation:
            logging.warn('Control response larger than saturation')

        return response
