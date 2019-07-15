# Holds class definitions for components for the control functions


# Limit function is bypassed if both upper and lower bound are set to zero
def limit(x, lower_lim, upper_lim):

    if lower_lim == 0 and upper_lim == 0:
        return x

    else:
        return max(lower_lim, min(x, upper_lim))


class Integrator:  # Integrator is internally limited, set limits to zero for unlimited operation

    def __init__(self, Rval, T, upper_lim, lower_lim, dt):

        self.Rval = Rval
        self.upper_lim = upper_lim
        self.lower_lim = lower_lim
        self.k = dt/(2*T)
        self.old_x = 0
        self.old_out = 0
        self.output = 0

    def reset(self):

        self.old_x = 0
        self.old_out = self.Rval
        self.output = self.Rval

    def calculate(self, x):

        result = self.k * (x + self.old_x) + self.old_out
        self.output = limit(result, self.lower_lim, self.upper_lim)
        self.old_x = x
        self.old_out = self.output

        return self.output


class RealPole:

    def __init__(self, T, G, dt, upper_lim, lower_lim, Rval):
        self.G = G
        self.K = (2 * T) / dt
        self.x_old = 0
        self.out_old = 0
        self.upper_lim = upper_lim
        self.lower_lim = lower_lim
        self.output = 0
        self.Rval = Rval

    def reset(self):
        self.x_old = 0
        self.out_old = self.Rval
        self.output = self.Rval

    def calculate(self, x):
        result = (1 / (1 + self.K)) * (self.x_old + x) + ((self.K - 1) / (self.K + 1)) * self.out_old
        result = self.G * result

        self.output = limit(result, self.lower_lim, self.upper_lim)

        self.x_old = x
        self.out_old = self.output

        return self.output


class MPPT:

    def __init__(self, strtup, vstep, sample):
        self.state = 0
        self.v_hist = 0
        self.i_hist = 0
        self.strtup = strtup
        self.vstep = vstep
        self.sample = sample
        self.last_sample = 0
        self.rst_time = 0
        self.Vmppt = 0

    def reset(self, time):
        self.rst_time = time

    def set_mode(self, time):
        if (time-self.rst_time) < self.strtup:
            self.state = 0
        else:
            self.state = 1

    def set_vmppt(self, v, i):
        if self.state == 0:
            self.Vmppt = (self.v_hist + v) / 2
            self.v_hist = v
            self.i_hist = i
        else:
            if (v*i) > (self.v_hist*self.i_hist):
                self.Vmppt = self.Vmppt + self.vstep
            elif (v*i) < (self.v_hist*self.i_hist):
                self.Vmppt = self.Vmppt - self.vstep
            else:
                self.Vmppt = self.Vmppt

            self.v_hist = v
            self.i_hist = i

    def calculate(self, time, v, i):
        if (time - self.last_sample) >= self.sample:
            self.set_mode(time)
            self.set_vmppt(v, i)


class PiControlVDC:

    def __init__(self, KiVDC, KpVDC, VDCmax, VDCmin):
        self.KiVDC = KiVDC
        self.KpVDC = KpVDC
        self.VDCmax = VDCmax
        self.VDCmin = VDCmin
        self.fb_loop = 0
        self.integrator = Integrator(0, 1, self.VDCmax, self.VDCmin, dt)

    def calculate(self, VDCerr, block8, reset8):

        integrator_input = (VDCerr + self.fb_loop) * self.KiVDC

        if reset8 != 0:
            self.fb_loop = 0
            self.integrator.reset()
            integrator_output = 0
        else:
            integrator_output = self.integrator.calculate(integrator_input)

        unlimited_PrefPV = (VDCerr * block8 * self.KpVDC) + integrator_output

        PrefPV = limit(unlimited_PrefPV, self.VDCmin, self.VDCmax)
        self.fb_loop = PrefPV - unlimited_PrefPV

        return PrefPV
