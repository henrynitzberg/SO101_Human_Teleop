# Thanks, Claude!

import math


def _smoothing_factor(t_e, cutoff):
    r = 2 * math.pi * cutoff * t_e
    return r / (r + 1)


class OneEuroFilter:
    """Adaptive low-pass filter for noisy real-time signals (hand/pointer/
    gesture tracking). Smooths hard when the signal is nearly still (kills
    idle jitter) and automatically smooths less when it's moving fast (stays
    responsive), rather than trading one off against the other with a single
    fixed time constant.

    Casiez, G., Roussel, N. and Vogel, D. 2012. 1(euro) Filter: A Simple
    Speed-based Low-pass Filter for Noisy Input in Interactive Systems.
    Proceedings of the SIGCHI Conference on Human Factors in Computing
    Systems (CHI '12). https://cristal.univ-lille.fr/~casiez/1euro/

    Tuning (per the paper): start with beta=0 and lower min_cutoff until
    jitter at rest is acceptable, then raise beta until lag during fast
    movement is acceptable.
    """

    def __init__(self, t0, x0, dx0=0.0, min_cutoff=1.0, beta=0.0, d_cutoff=1.0):
        self.min_cutoff = min_cutoff
        self.beta = beta
        self.d_cutoff = d_cutoff
        self.x_prev = x0
        self.dx_prev = dx0
        self.t_prev = t0

    def __call__(self, t, x):
        t_e = t - self.t_prev
        if t_e <= 0:
            return self.x_prev

        a_d = _smoothing_factor(t_e, self.d_cutoff)
        dx = (x - self.x_prev) / t_e
        dx_hat = a_d * dx + (1 - a_d) * self.dx_prev

        cutoff = self.min_cutoff + self.beta * abs(dx_hat)
        a = _smoothing_factor(t_e, cutoff)
        x_hat = a * x + (1 - a) * self.x_prev

        self.x_prev = x_hat
        self.dx_prev = dx_hat
        self.t_prev = t
        return x_hat
