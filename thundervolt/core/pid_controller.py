import logging
import numpy as np

class ButterworthSecondOrder:
    """
    Implementation of Butterworth second order low-pass filter

    A generic digital filter follows the relation

        a0 * y[k] = sum(bi * x[k - i]) - sum(aj * y[k - j])

    Where

        x[k] - measurement at instant k
        y[k] - filtered signal at instant k

    The Butterworth filter have the special property of being a
    maximally flat magnitude filter, in other words, is the best
    filter that doesn't present distortions around the cutoff
    frequency

    The formula for the continuos coefficients of the Butterworth
    filter is available here:

    https://en.wikipedia.org/wiki/Butterworth_filter

    The discrete version were computed with the Tustin method:

    https://en.wikipedia.org/wiki/Bilinear_transform

    """

    def __init__(self, cutoff_frequency, sampling_frequency=1.0):
        """
        Create a second order Butterworth filter

        Args:
            cutoff_frequency (float): Low-pass cutoff frequency in Hz
            sampling_frequency (float, optional): Sampling frequency in Hz.
                Defaults to 1.0.
        """
        w = cutoff_frequency/sampling_frequency

        if w > 0.5:
            logging.warn('Filter cutoff frequency larger than Nyquist frequency')

        b0 = 1
        b1 = 2
        b2 = 1

        a0 = 1 + 2*np.sqrt(2)/w + 4/w**2
        a1 = 2 - 8/w**2
        a2 = 1 - 2*np.sqrt(2)/w + 4/w**2

        self.x = np.array([0,0,0])
        self.y = np.array([0,0])

        self.a = np.array([a2,a1])/a0
        self.b = np.array([b2,b1,b0])/a0

    def update(self, x0):
        """
        Produces a new value from measured data

        Args:
            x0 (float): Last measure

        Returns:
            float: Filtered value
        """

        self.x = np.append(self.x, x0)[1:]
        y0 = np.dot(self.x, self.b) - np.dot(self.y, self.a)

        self.y = np.append(self.y, y0)[1:]
        return y0

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

        # cutoff frequency is 1/5 of sampling frequency
        self.dedt_filter = ButterworthSecondOrder(1/5)

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

        dedt = self.dedt_filter.update((error - self.prev_error)*self.freq)

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
