# Holds class definitions for components for the control functions

import math
import numpy


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

    def __init__(self, KiVDC, KpVDC, VDCmax, VDCmin, dt):
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


class VarFreqRampGen:

    def __init__(self, max_ramp, min_ramp, T, dt):
        self.min = min_ramp
        self.max = max_ramp
        self.integrator = Integrator(min_ramp, T, max_ramp, min_ramp, dt)
        self.output = min_ramp

    def calculate(self, x):

        self.output = self.integrator.calculate(x)

        if self.output >= self.max:
            self.integrator.reset()

        return self.output


class PLL:
    def __init__(self, dt, FDEV8):
        self.SA8 = 0
        self.SB8 = 0
        self.SC8 = 0
        self.CA8 = 0
        self.CB8 = 0
        self.CC8 = 0
        self.erra8 = 0
        self.errb8 = 0
        self.errc8 = 0
        self.integrator_UDO = Integrator(0, 1.0, 0, 0, dt)
        self.integrator_DWO = Integrator(0, 1.0, 0, 0, dt)
        self.ramp = VarFreqRampGen((2 * math.pi), 0, 1.0, dt)
        self.DWOMAX8 = (2 * math.pi) * FDEV8
        self.DWOMIN8 = -1 * self.DWOMAX8
        self.ERR_58 = 0
        self.OMEGA8 = 0
        self.angPLL8 = 0

    def calculate(self, VSAA8, VSBA8, VSCA8, Vbase8, FACT8, wA8):
        N4Apu8 = VSAA8 / Vbase8
        N5Apu8 = VSBA8 / Vbase8
        N6Apu8 = VSCA8 / Vbase8
        UDO8 = ((self.SA8 * self.erra8) + (self.SB8 * self.errb8) + (self.SC8 * self.errc8)) * (800 / 3)
        UO8 = self.integrator_UDO.calculate(UDO8)
        self.erra8 = N4Apu8 - (UO8 * self.SA8)
        self.errb8 = N5Apu8 - (UO8 * self.SB8)
        self.errc8 = N6Apu8 - (UO8 * self.SC8)
        self.ERR_58 = ((self.erra8 * self.CA8) + (self.errb8 * self.CB8) + (self.errc8 * self.CC8)) * (2 / 3)
        DWOLIM8 = limit(self.integrator_DWO.calculate(self.ERR_58 * 20000), self.DWOMIN8, self.DWOMAX8)
        self.OMEGA8 = (DWOLIM8 * FACT8) + wA8
        self.angPLL8 = self.ramp.calculate(self.OMEGA8 + self.ERR_58)
        ANGA8 = self.angPLL8
        ANGB8 = self.angPLL8 - (2 * math.pi / 3)
        ANGC8 = self.angPLL8 + (2 * math.pi / 3)
        self.SA8 = math.sin(ANGA8)
        self.SB8 = math.sin(ANGB8)
        self.SC8 = math.sin(ANGC8)
        self.CA8 = math.cos(ANGA8)
        self.CB8 = math.cos(ANGB8)
        self.CC8 = math.cos(ANGC8)

        return self.angPLL8, self.OMEGA8, self.ERR_58


class ABC2DQO:

    def __init__(self, Vq_lags_Vd):
        self.Vq_lags_Vd = Vq_lags_Vd
        self.con = 2*math.pi/3

    def calculate(self, phase, Va, Vb, Vc):

        if self.Vq_lags_Vd:
            transform = numpy.array([[math.sin(phase), math.sin(phase-self.con), math.sin(phase+self.con)],
                                    [math.cos(phase), math.cos(phase-self.con), math.cos(phase+self.con)],
                                    [(1/2), (1/2), (1/2)]])
        else:
            transform = numpy.array([[math.cos(phase), math.cos(phase - self.con), math.cos(phase + self.con)],
                                     [math.sin(phase), math.sin(phase - self.con), math.sin(phase + self.con)],
                                     [(1 / 2), (1 / 2), (1 / 2)]])

        Vdqo = (2/3) * numpy.matmul(transform, numpy.array([[Va], [Vb], [Vc]]))

        return Vdqo[0, 0], Vdqo[1, 0], Vdqo[2, 0]      # Returns Vd, Vq, Vo


class EnhancedPLL:

    def __init__(self, dt, FDEV8):
        self.pll = PLL(dt, FDEV8)
        self.abc_to_dqo_V = ABC2DQO(True)
        self.abc_to_dqo_I = ABC2DQO(True)
        # self.angPLL8 = 0
        # self.OMEAGA8 = 0
        # self.ERR_58 = 0

    def calculate(self, VSAA8, VSBA8, VSCA8, Vbase8, FACT8, wA8, IAA8, IBA8, ICA8):
        angPLL8, OMEAGA8, ERR_58 = self.pll.calculate(VSAA8, VSBA8, VSCA8, Vbase8, FACT8, wA8)
        VsdA8, VsqA8, VsoA8 = self.abc_to_dqo_V.calculate(angPLL8, VSAA8, VSBA8, VSCA8)
        IsdA8, IsqA8, IsoA8 = self.abc_to_dqo_I.calculate(angPLL8, -1*IAA8, -1*IBA8, -1*ICA8)

        return VsdA8, VsqA8, IsdA8, IsqA8, OMEAGA8, ERR_58