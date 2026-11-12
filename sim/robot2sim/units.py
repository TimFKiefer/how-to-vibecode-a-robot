# SPDX-License-Identifier: MIT
"""Conversions between the CAD units and MuJoCo SI units.

The CAD script and the viewer use millimetres, degrees, grams and kg·cm.
MuJoCo uses metres, radians, kilograms and N·m.

1 kg·cm = 0.0980665 N·m  (1 kgf · 1 cm, with g = 9.80665 m/s²).
Servo speeds in rpm: 1 rpm = 2π / 60 rad/s.
"""

import math

MM_TO_M = 1e-3
G_TO_KG = 1e-3
DEG_TO_RAD = math.pi / 180.0
KGCM_TO_NM = 0.0980665
GRAVITY = 9.80665


def mm_to_m(x):
    return x * MM_TO_M


def m_to_mm(x):
    return x / MM_TO_M


def g_to_kg(x):
    return x * G_TO_KG


def kg_to_g(x):
    return x / G_TO_KG


def deg_to_rad(x):
    return x * DEG_TO_RAD


def rad_to_deg(x):
    return x / DEG_TO_RAD


def kgcm_to_nm(x):
    return x * KGCM_TO_NM


def nm_to_kgcm(x):
    return x / KGCM_TO_NM


def rpm_to_rad_s(x):
    return x * 2.0 * math.pi / 60.0

