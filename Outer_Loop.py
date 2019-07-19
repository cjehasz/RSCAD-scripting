# This module is meant to replace the DC Bus Control system in PV controls

# Import section
import socket
import struct
from components import *


# ------------------- FUNCTION AND CLASS DEFINITIONS ----------------------------------------------------------------


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

    # if MPPTRSET != 0:
    #     mppt_blk.reset(time)
    #     mppt_blk.calculate(time, VPVF, Ika)
    # else:
    #     mppt_blk.calculate(time, VPVF, Ika)

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

# ------------------- MAIN CODE ------------------------------------------------------------------------------------

dt = 0.0002    # 0.2e-3
t = 0.0         # time counter

# Enter constants (Slider values in RTDS)

KpVDC = 1.0     # Kp/Ki from VDC pi control block
KiVDC = 5.0
VDCmax = 5.0    # VDC Min/Max in VDC PI control block
VDCmin = -5.0
strtup = 5      # MPPT waiting period, in seconds
samp = 0.250    # MPPT sampling period, in seconds
vstep = 2       # MPPT voltage step
rp_T = 0.001    # Real pole block T value
rp_G = 1.0      # Real pole block G value
rp_Rval = 0     # Real pole reset value

# Build Control Components ----------------------------------------

# For dc_bus_vc:
mppt_blk = MPPT(strtup, vstep, samp)
pi_ctl_vdc = PiControlVDC(KiVDC, KpVDC, VDCmax, VDCmin, dt)
real_pole_vpvf = RealPole(rp_T, rp_G, dt, 0, 0, rp_Rval)
real_pole_ika = RealPole(rp_T, rp_G, dt, 0, 0, rp_Rval)

# # For ac_bus_vc
# pi_ctl_vac = PiControlVDC(5, 0.025, 5, -5)

# # For get_IsqrefV38
# pi_ctl_q = PiControlVDC(5, 1, 5, -5)

# Network Code ----------------------------------------------------

IPAddr = socket.gethostbyname(socket.gethostname())
# print("Your Computer IP Address is: " + IPAddr)

# RTDS import/export
UDP_IP_ctr = IPAddr
UDP_IP_rtds = '10.16.158.36'
UDP_PORT = 12345
sock = socket.socket(socket.AF_INET,  # Internet
                     socket.SOCK_DGRAM)  # UDP
sock.bind((UDP_IP_ctr, UDP_PORT))

# sock.settimeout(30)

print(IPAddr)

while True:
    # Receive Data and Unpack into list
    data, addr = sock.recvfrom(1024)
    F = struct.unpack('>ffiiffi', data)      # f = float, i = int; order is same as as variables sent from RTDS

    # Assign variables from packet
    VDCA8 = F[0]
    IPVAxxx = F[1]
    MPPTRSET = F[2]
    MPPT_CTL = F[3]
    VMPPA = F[4]
    block8 = F[5]
    reset8 = F[6]

    # Calculate control signal
    Isdref_V28x = dc_bus_vc(VDCA8, IPVAxxx, MPPTRSET, MPPT_CTL, VMPPA, block8, reset8, t)

    # Send ctl signal back
    y = struct.pack('>f', Isdref_V28x)
    sock.sendto(y, (UDP_IP_rtds, UDP_PORT))

    # increment time counter
    t = t + dt

    # Repeat loop (set socket timeout or other exit condition?)

