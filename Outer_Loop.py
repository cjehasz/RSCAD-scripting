# This module is meant to replace the DC Bus Control system in PV controls

# Limit function is bypassed if both upper and lower bound are set to zero


def limit(x, lower_lim, upper_lim):

    if lower_lim != 0 and upper_lim != 0:
        return max(lower_lim, min(x, upper_lim))

    else:
        return x


class Integrator:  # Integrator is internally limited, set limits to zero for unlimited operation

    def __init__(self, Rval, T, upper_lim, lower_lim, dt):

        self.Rval = Rval
        self.upper_lim = upper_lim
        self.lower_lim = lower_lim           # future getTimestamp()?
        self.k = dt/(2*T)
        self.old_x = 0
        self.old_out = 0
        self.output = 0

    def reset(self):

        self.old_x = 0
        self.old_out = 0
        self.output = 0

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
            self.integrator.reset()
            integrator_output = 0
        else:
            integrator_output = self.integrator.calculate(integrator_input)

        unlimited_PrefPV = (VDCerr * block8 * self.KpVDC) + integrator_output

        PrefPV = limit(unlimited_PrefPV, self.VDCmin, self.VDCmax)
        self.fb_loop = PrefPV - unlimited_PrefPV

        return PrefPV


dt = 0.001                              # Find out how to get timestep

# Build Control Components

mppt_blk = MPPT(5, 2, 0.250)
pi_ctl_vdc = PiControlVDC(5, 1, 5, -5)
pi_ctl_vac = PiControlVDC(5, 0.025, 5, -5)
pi_ctl_q = PiControlVDC(5, 1, 5, -5)
real_pole_vpvf = RealPole(0.001, 1, dt, 0, 0, 0)
real_pole_ika = RealPole(0.001, 1, dt, 0, 0, 0)


def dc_bus_vc(VDCA8, IPVAxxx, MPPTRSET, MPPT_CTL, VMPPA, block8, reset8, time):

    VPVF = real_pole_vpvf.calculate(VDCA8)
    Ika = real_pole_ika.calculate(IPVAxxx)

    # MPPT Calculation line

    if MPPTRSET != 0:
        mppt_blk.reset(time)
        mppt_blk.set_mode(time)
        mppt_blk.set_vmppt(VPVF, Ika)
    else:
        mppt_blk.set_mode(time)
        mppt_blk.set_vmppt(VPVF, Ika)

    Mode = mppt_blk.state
    VMPPD = mppt_blk.Vmppt

    if Mode == 0 or MPPT_CTL == 0:
        VDCREF = VMPPA
    else:
        VDCREF = VMPPD

    VDCerr = (VPVF**2 - VDCREF**2) * block8

    # PI Control VDC

    PrefPV = pi_ctl_vdc.calculate(VDCerr, block8, reset8)

    # Post PI Control VDC Calculations

    Vbase8 = (0.48*(2**0.5)) / (3**0.5)

    Isdref_V28x = PrefPV / (Vbase8 * 1.5)

    return Isdref_V28x

# Git Test

# def get_Isqref_V38(QSrefA8, Qmeas8, block8, reset8):
#
#     Qerr8 = (QSrefA8 - Qmeas8) * block8
#     Isqref_V38 = -1 * pi_ctl_q.calculate(Qerr8, block8, reset8)
#
#     return Isqref_V38


# def get_Isqref_V28(Qreftest8, VsdAf8):
#
#     QSrefA8 = limit(Qreftest8, -0.05, 0.05)
#     Isqref_V28 = ((-2/3) / VsdAf8) * QSrefA8
#
#     return Isqref_V28

# def ac_bus_vc(VsdAf8, Vsdref8, block8, reset8):
#
#     VACerr8 = (Vsdref8 - VsdAf8) * block8
#     QrefPV8 = pi_ctl_vac.calculate(VACerr8, block8, reset8)
#
#     Isqref_V38xxxx = QrefPV8 / (-1.5 * Vsdref8)
#
#     return Isqref_V38xxxx

# def reference_selection(select, Isdref_V28x, Isqref_V18, Isqref_V28, Isqref_V38, Isqref_V38xxxx, block8):
#
#     Isdrefongrid8 = block8 *  Isdref_V28x
#     Isqref_bank = [Isqref_V18, Isqref_V28, Isqref_V38, Isqref_V38xxxx]
#
#     if select > 4 or select < 0:
#         return 0, 0
#     else:
#         return Isdrefongrid8, (block8 * Isqref_bank[(select-1)])

