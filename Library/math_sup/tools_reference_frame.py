# -*- coding: utf-8 -*-
"""
Created on Sat Jan 18 14:34:09 2020

@author: EO
"""
import numpy as np

mu = 3.986044418e5
twopi = 2.0 * np.pi
deg2rad = np.pi / 180.0
omega_earth = 7.2921150 * 1e-5  # rad/s
a2 = 40680631.6     # Equatorial radius
b2 = 40408296.0     # Polar radius
radius_earth = 6378.137  # km
earth_flat = 1.0 / 298.257223563
earth_e2 = earth_flat * (2 - earth_flat)
geod_tolerance = 1e-10  # rad


def rotationY(bfr, theta):
    temp = np.zeros(3)
    temp[0] = np.cos(theta)*bfr[0] - np.sin(theta)*bfr[2]
    temp[1] = bfr[1]
    temp[2] = np.sin(theta)*bfr[0] + np.cos(theta)*bfr[2]
    return temp


def eci_to_geodetic(current_position_i, current_sideral):
    r = np.sqrt(current_position_i[0] ** 2 + current_position_i[1] ** 2)
    long = fmod2(np.arctan2(current_position_i[1], current_position_i[0]) - current_sideral)
    lat = np.arctan2(current_position_i[2], r)
    flag_iteration = True
    c = 1
    while flag_iteration:
        phi = lat
        c = 1 / np.sqrt(1 - earth_e2 * np.sin(phi) * np.sin(phi))
        lat = np.arctan2(current_position_i[2] + radius_earth * c
                         * earth_e2 * np.sin(phi) * 1000, r)
        if (np.abs(lat - phi)) <= geod_tolerance:
            flag_iteration = False

    alt = r / np.cos(lat) - radius_earth * c * 1000  # *metros

    if lat > np.pi / 2:
        lat -= twopi
    return alt, long, lat


def geodetic_to_ecef(tar_alt, tar_long, tar_lat):
    a2cos2 = a2 * np.cos(tar_lat) ** 2
    b2sin2 = b2 * np.sin(tar_lat) ** 2
    N = a2/np.sqrt(a2cos2 + b2sin2)
    x = (N + tar_alt) * np.cos(tar_lat) * np.cos(tar_long)
    y = (N + tar_alt) * np.cos(tar_lat) * np.sin(tar_long)
    z = (N * b2 / a2 + tar_alt) * np.sin(tar_lat)
    return np.array([x, y, z])


def rotationZ(bfr, theta):
    temp = np.zeros(3)
    temp[0] = np.cos(theta)*bfr[0] + np.sin(theta)*bfr[1]
    temp[1] = -np.sin(theta)*bfr[0] + np.cos(theta)*bfr[1]
    temp[2] = bfr[2]
    return temp


def fmod2(x):
    if x > np.pi:
        x -= twopi
    elif x < -np.pi:
        x += twopi
    else:
        x = x
    return x

def unitVector(v):
    norm = np.linalg.norm(v) # norm = |v|
    if norm!=0:
        u = v/norm           # unit vector u = v / |v|
    else:
        u = v                # u = v = 0
    return u

def skewMatrix(a):
    ax = np.zeros((3, 3))

    ax[1, 0] = a[2]
    ax[2, 0] = -a[1]

    ax[0, 1] = -a[2]
    ax[0, 2] = a[1]

    ax[2, 1] = a[0]
    ax[1, 2] = -a[0]
    return ax

# from Julian date Calculate DecimalYear
def JdToDecyear(jd):
    # --------------- find year and days of the year ---------------
    temp = jd - 2415019.5
    tu = temp / 365.25
    year = 1900 + np.floor(tu)
    leapyrs = np.floor((year - 1901) * 0.25)

    # optional nudge by 8.64x10-7 sec to get even outputs
    days = temp - ((year - 1900) * 365.0 + leapyrs) + 0.00000000001

    # ------------ check for case of beginning of a year -----------
    if days < 1.0:
        year = year - 1
        leapyrs = np.floor((year - 1901) * 0.25)
        days = temp - ((year - 1900) * 365.0 + leapyrs)

    decyear = year + days / 365.25
    return decyear


# def gstime(jd):
#     jd0 = jd[0]
#     utc = jd[1]
#     tut1 = (jd0 - 2451545.0) / 36525.0
#     GST0_time = -6.2e-6 * tut1 ** 3 + 0.093104 * tut1 ** 2 + 8640184.812866 * tut1 + 24110.54841  # sec
#     GST0_rad = (GST0_time * deg2rad / 240.0)  # 360/86400 = 1/240, to deg, to rad
#     GST_rad = (GST0_rad + omega_earth * utc / 3600) % twopi
#     #  ------------------------ check quadrants ---------------------
#     if GST_rad < 0.0:
#         GST_rad += twopi
#     return GST_rad


def jday(year, mon, day, hr, minute, sec):
    jd0 = 367.0 * year - 7.0 * (year + ((mon + 9.0) // 12.0)) * 0.25 // 1.0 + 275.0 * mon // 9.0 + day + 1721013.5
    utc = ((sec / 60.0 + minute) / 60.0 + hr)  # utc in hours#
    return [jd0, utc]


def gstime(jdut1):
    tut1 = (jdut1 - 2451545.0) / 36525.0
    temp = -6.2e-6 * tut1 * tut1 * tut1 + 0.093104 * tut1 * tut1 + \
           (876600.0 * 3600 + 8640184.812866) * tut1 + 67310.54841  # sec
    temp = (temp * deg2rad / 240.0) % twopi  # 360/86400 = 1/240, to deg, to rad

    #  ------------------------ check quadrants ---------------------
    if temp < 0.0:
        temp += twopi

    return temp


def getOE(r, v):
    h = np.cross(r, v)
    rNorm = np.linalg.norm(r)
    vNorm = np.linalg.norm(v)
    eVect = np.cross(v, h) / mu - r / rNorm
    e = np.linalg.norm(eVect)
    a = 1 / (2 / rNorm - vNorm ** 2 / mu)
    ie = eVect / e
    ih = h / np.linalg.norm(h)
    ip = np.cross(ih, ie)
    C31 = ih[0]
    C32 = ih[1]
    C33 = ih[2]
    C13 = ie[2]
    C23 = ip[2]
    RAAN = np.atan2(C31, - C32)
    i = np.arccos(C33)
    w = np.arctan2(C13, C23)
    # n = np.sqrt(mu/a**3)
    sigma0 = np.dot(r, v) / np.sqrt(mu)
    E0 = np.arctan2(sigma0 / np.sqrt(a), 1 - rNorm / a)
    M = E0 - e * np.sin(E0)
    # f = 2*np.arctan(np.sqrt((1.0 + e)/(1.0 - e))*np.tan(0.5*E0))
    return a, e, np.rad2deg(i), np.rad2deg(RAAN), np.rad2deg(w), np.rad2deg(M)
