#!/usr/bin/env python3
"""
PMSM FOC Simulation Verification & Bug Diagnosis
=================================================
Independently verifies the PMSM FOC Simulink model by implementing the
exact same equations in Python, then identifies and demonstrates 3 bugs.

Bugs found in the Simulink model:
  1. speed_ref (RPM) vs omega_m (rad/s) units mismatch in Speed PI
  2. Derivative block for omega_m (noisy, unnecessary)
  3. Back-EMF > Vdc at 1000 RPM (motor cannot reach target speed)

Usage:
    python3 verify_foc_simulation.py
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import os

# ============================================================
# Motor Parameters (from pmsm_foc_simscape.m)
# ============================================================
Rs       = 0.5        # Stator resistance [Ohm]
Ld       = 1.4e-3     # d-axis inductance [H]
Lq       = 1.4e-3     # q-axis inductance [H]
flux_pm  = 0.0577     # PM flux linkage [Wb]
pp       = 4          # Pole pairs
J        = 1.74e-5    # Rotor inertia [kg*m^2]
B_damp   = 1e-4       # Viscous damping [N*m*s]

# Controller
Kp_id = 5.0;  Ki_id = 1000.0
Kp_iq = 5.0;  Ki_iq = 1000.0
Kp_speed = 0.5; Ki_speed = 10.0
iq_max = 10.0

# Simulation
dt    = 1e-5
t_end = 0.5
N     = int(t_end / dt) + 1

# ============================================================
# Transform Functions
# ============================================================
def clarke(ia, ib, ic):
    return (2/3*(ia - 0.5*ib - 0.5*ic),
            2/3*(np.sqrt(3)/2*ib - np.sqrt(3)/2*ic))

def park(alpha, beta, theta):
    return (alpha*np.cos(theta) + beta*np.sin(theta),
           -alpha*np.sin(theta) + beta*np.cos(theta))

def inv_park(d, q, theta):
    return (d*np.cos(theta) - q*np.sin(theta),
            d*np.sin(theta) + q*np.cos(theta))

def inv_clarke_3ph(alpha, beta):
    return alpha, -0.5*alpha + np.sqrt(3)/2*beta, -0.5*alpha - np.sqrt(3)/2*beta

def svpwm(v_alpha, v_beta, vdc):
    va = v_alpha
    vb = -0.5*v_alpha + np.sqrt(3)/2*v_beta
    vc = -0.5*v_alpha - np.sqrt(3)/2*v_beta
    half = vdc / 2.0
    va_n, vb_n, vc_n = va/half, vb/half, vc/half
    voff = -(max(va_n,vb_n,vc_n) + min(va_n,vb_n,vc_n)) / 2
    return (np.clip((va_n+voff+1)/2, 0, 1),
            np.clip((vb_n+voff+1)/2, 0, 1),
            np.clip((vc_n+voff+1)/2, 0, 1))

# ============================================================
# PMSM Simulation Core
# ============================================================
def simulate(speed_ref_rpm, vdc, use_rads_conversion=True):
    """Run closed-loop PMSM FOC simulation.

    Args:
        speed_ref_rpm: Target speed in RPM (ramp over 0.1s)
        vdc: DC bus voltage
        use_rads_conversion: If True, convert speed_ref to rad/s before PI.
                            If False, compare RPM to rad/s (the bug).
    """
    id_s = iq_s = wm = theta_m = 0.0
    int_spd = int_id = int_iq = 0.0

    t_arr  = np.zeros(N)
    wm_arr = np.zeros(N)
    id_arr = np.zeros(N)
    iq_arr = np.zeros(N)
    Te_arr = np.zeros(N)
    ref_arr = np.zeros(N)

    for n in range(N):
        t = n * dt
        t_arr[n] = t

        spd_ref = min(speed_ref_rpm / 0.1 * t, speed_ref_rpm)
        Te_load = 0.1 if t >= 0.3 else 0.0
        ref_arr[n] = spd_ref

        theta_e = pp * theta_m

        # FOC measurement: dq -> abc -> Clarke -> Park (roundtrip = identity)
        i_a, i_b, i_c = inv_clarke_3ph(*inv_park(id_s, iq_s, theta_e))
        i_alpha, i_beta = clarke(i_a, i_b, i_c)
        id_m, iq_m = park(i_alpha, i_beta, theta_e)

        # Speed PI
        if use_rads_conversion:
            spd_err = spd_ref * 2*np.pi/60 - wm
        else:
            spd_err = spd_ref - wm  # BUG: RPM vs rad/s

        int_spd += spd_err * dt
        iq_ref = np.clip(Kp_speed * spd_err + Ki_speed * int_spd, -iq_max, iq_max)

        # Current PIs
        id_err = 0.0 - id_m
        int_id += id_err * dt
        vd_cmd = Kp_id * id_err + Ki_id * int_id

        iq_err = iq_ref - iq_m
        int_iq += iq_err * dt
        vq_cmd = Kp_iq * iq_err + Ki_iq * int_iq

        # Inverse Park + SVPWM + Power Stage
        va, vb = inv_park(vd_cmd, vq_cmd, theta_e)
        da, db, dc = svpwm(va, vb, vdc)
        Va, Vb, Vc = da*vdc, db*vdc, dc*vdc

        # Plant: abc -> dq voltages -> derivatives
        v_al = 2/3*(Va - 0.5*Vb - 0.5*Vc)
        v_be = 2/3*(np.sqrt(3)/2*Vb - np.sqrt(3)/2*Vc)
        vd_p = v_al*np.cos(theta_e) + v_be*np.sin(theta_e)
        vq_p = -v_al*np.sin(theta_e) + v_be*np.cos(theta_e)

        we = pp * wm
        did = (vd_p - Rs*id_s + we*Lq*iq_s) / Ld
        diq = (vq_p - Rs*iq_s - we*Ld*id_s - we*flux_pm) / Lq
        Te = 1.5 * pp * (flux_pm*iq_s + (Ld-Lq)*id_s*iq_s)
        dwm = (Te - B_damp*wm - Te_load) / J
        dtm = wm

        wm_arr[n] = wm; id_arr[n] = id_m; iq_arr[n] = iq_m; Te_arr[n] = Te

        # Forward Euler
        id_s   += did * dt
        iq_s   += diq * dt
        wm     += dwm * dt
        theta_m = (theta_m + dtm*dt) % (2*np.pi)

    return dict(t=t_arr, wm=wm_arr, id=id_arr, iq=iq_arr, Te=Te_arr, ref=ref_arr)


def main():
    print("=" * 65)
    print("  PMSM FOC Verification & Bug Diagnosis")
    print("=" * 65)

    # --- Physical Limits Analysis ---
    Vdc_orig = 24.0
    V_max = Vdc_orig / np.sqrt(3)
    omega_m_target = 1000 * 2*np.pi/60
    omega_e_target = pp * omega_m_target
    bemf = omega_e_target * flux_pm

    print("\n--- Physical Limits Analysis ---")
    print(f"  Vdc = {Vdc_orig} V, Max inverter voltage = {V_max:.2f} V")
    print(f"  Back-EMF at 1000 RPM = {bemf:.2f} V")
    print(f"  >>> Back-EMF ({bemf:.1f}V) > V_max ({V_max:.1f}V): CANNOT REACH 1000 RPM!")
    max_rpm = V_max / flux_pm / pp * 60 / (2*np.pi)
    print(f"  Max achievable speed ~ {max_rpm:.0f} RPM (no-load, Vdc=24V)")

    print("\n--- Bug #1: RPM vs rad/s mismatch ---")
    print("  speed_ref(RPM) - omega_m(rad/s) → wrong units in Speed PI!")
    print("\n--- Bug #2: Derivative block for omega_m ---")
    print("  Uses d(theta_m)/dt, but motor already outputs omega_m directly.")
    print("\n--- Bug #3: Vdc=24V too low for 1000 RPM ---")
    print(f"  Need Vdc > {bemf*np.sqrt(3):.1f}V, or lower speed_ref to <{max_rpm:.0f} RPM")

    # --- Run 4 Simulations ---
    print("\n" + "=" * 65)
    sims = [
        ("BUGGY: RPM vs rad/s, Vdc=24V, 1000RPM", 1000, 24, False),
        ("Fix units only, Vdc=24V (saturated)",     1000, 24, True),
        ("ALL FIXED: units + Vdc=48V",              1000, 48, True),
        ("Fix units, speed=500RPM, Vdc=24V",         500, 24, True),
    ]
    results = []
    for label, rpm, vdc, conv in sims:
        print(f"\n  [{label}]")
        r = simulate(rpm, vdc, conv)
        rpm_final = r['wm'][-1] * 60/(2*np.pi)
        print(f"    Final: speed={rpm_final:.1f} RPM, id={r['id'][-1]:.3f} A, "
              f"iq={r['iq'][-1]:.3f} A, Te={r['Te'][-1]:.4f} Nm")
        results.append((label, r, rpm))

    # --- Check "all fixed" simulation ---
    r_ok = results[2][1]
    rpm_ok = r_ok['wm'][-1] * 60/(2*np.pi)
    print(f"\n--- Verification (ALL FIXED case) ---")
    ok = True
    if abs(rpm_ok - 1000)/1000 < 0.05:
        print(f"  [PASS] Speed={rpm_ok:.1f} RPM (within 5% of 1000)")
    else:
        print(f"  [FAIL] Speed={rpm_ok:.1f} RPM"); ok = False
    if abs(r_ok['id'][-1]) < 0.5:
        print(f"  [PASS] id={r_ok['id'][-1]:.3f} A (near 0)")
    else:
        print(f"  [FAIL] id={r_ok['id'][-1]:.3f} A"); ok = False
    te_ss = np.mean(r_ok['Te'][-5000:])
    if abs(te_ss - 0.1) < 0.05:
        print(f"  [PASS] Steady-state Te={te_ss:.4f} Nm (≈load)")
    else:
        print(f"  [WARN] Te={te_ss:.4f} Nm")
    if ok:
        print("  >>> ALL CHECKS PASSED!")

    # --- Plots ---
    fig, axes = plt.subplots(4, 3, figsize=(16, 14))
    fig.suptitle('PMSM FOC Verification: Bug Diagnosis', fontsize=14, fontweight='bold')

    for row, (label, data, target) in enumerate(results):
        t_ms = data['t'] * 1000

        ax = axes[row, 0]
        ax.plot(t_ms, data['wm']*60/(2*np.pi), 'b-', lw=0.5, label='actual')
        ax.axhline(y=target, color='r', ls='--', lw=1, label=f'ref={target}RPM')
        ax.set_ylabel('Speed [RPM]'); ax.set_title(f'{label}: Speed')
        ax.legend(fontsize=7, loc='lower right'); ax.grid(True, alpha=0.3)

        ax = axes[row, 1]
        ax.plot(t_ms, data['id'], 'b-', lw=0.5)
        ax.axhline(y=0, color='r', ls='--', lw=1)
        ax.set_ylabel('id [A]'); ax.set_title(f'{label}: id')
        ax.grid(True, alpha=0.3)

        ax = axes[row, 2]
        ax.plot(t_ms, data['Te'], 'b-', lw=0.5, label='Te')
        load_sig = np.where(data['t'] >= 0.3, 0.1, 0.0)
        ax.plot(t_ms, load_sig, 'r--', lw=1, label='load')
        ax.set_ylabel('Torque [Nm]'); ax.set_title(f'{label}: Torque')
        ax.legend(fontsize=7); ax.grid(True, alpha=0.3)

        if row == 3:
            for c in range(3): axes[row, c].set_xlabel('Time [ms]')

    plt.tight_layout()
    out_path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                            'foc_verification_results.png')
    plt.savefig(out_path, dpi=150)
    print(f"\nPlot saved to: {out_path}")

    # --- Summary ---
    print("\n" + "=" * 65)
    print("  FIXES NEEDED IN SIMULINK MODEL")
    print("=" * 65)
    print("""
  Fix #1: Add RPM-to-rad/s conversion in FOC Controller
    → Add Gain block (2*pi/60) on speed_ref input before Speed PI

  Fix #2: Pass omega_m directly from motor, remove Derivative block
    → Add input port 5 to Measurements for omega_m
    → Connect PMSM Motor/4 -> Measurements/5 (pass-through)

  Fix #3: Increase Vdc from 24V to 48V
    → Or lower speed_ref to 500 RPM
""")

if __name__ == '__main__':
    main()
