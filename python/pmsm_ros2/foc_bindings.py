"""
pybind11 bindings for the C++ FOC controller.

Build with:
    cd cpp && mkdir build && cd build
    cmake .. -DBUILD_PYTHON_BINDINGS=ON
    make
"""

try:
    # Try to import the compiled C++ module
    from _pmsm_cpp import (
        FOCController,
        FOCConfig,
        FOCOutput,
        clarke,
        park,
        inv_park,
        svpwm,
    )
except ImportError:
    # Fallback: pure Python implementation for development/testing
    import math

    class FOCConfig:
        """FOC controller configuration (Python fallback)."""
        def __init__(self):
            self.Kp_id = 5.0
            self.Ki_id = 1000.0
            self.Kp_iq = 5.0
            self.Ki_iq = 1000.0
            self.Kp_speed = 0.5
            self.Ki_speed = 10.0
            self.Rs = 0.5
            self.Ld = 1.4e-3
            self.Lq = 1.4e-3
            self.flux_pm = 0.0577
            self.pole_pairs = 4
            self.iq_max = 10.0
            self.id_max = 10.0
            self.Vdc = 24.0
            self.Ts = 50e-6

    class FOCOutput:
        """FOC controller output (Python fallback)."""
        def __init__(self):
            self.duty_a = 0.5
            self.duty_b = 0.5
            self.duty_c = 0.5
            self.id_meas = 0.0
            self.iq_meas = 0.0
            self.vd = 0.0
            self.vq = 0.0
            self.iq_ref = 0.0

    def clarke(ia, ib, ic):
        k = 2.0 / 3.0
        sqrt3_2 = math.sqrt(3) / 2
        alpha = k * (ia - 0.5 * ib - 0.5 * ic)
        beta = k * (sqrt3_2 * ib - sqrt3_2 * ic)
        return alpha, beta

    def park(alpha, beta, theta_e):
        cos_t = math.cos(theta_e)
        sin_t = math.sin(theta_e)
        d = alpha * cos_t + beta * sin_t
        q = -alpha * sin_t + beta * cos_t
        return d, q

    def inv_park(d, q, theta_e):
        cos_t = math.cos(theta_e)
        sin_t = math.sin(theta_e)
        alpha = d * cos_t - q * sin_t
        beta = d * sin_t + q * cos_t
        return alpha, beta

    def svpwm(v_alpha, v_beta, Vdc):
        sqrt3_2 = math.sqrt(3) / 2
        va = v_alpha
        vb = -0.5 * v_alpha + sqrt3_2 * v_beta
        vc = -0.5 * v_alpha - sqrt3_2 * v_beta
        half_Vdc = Vdc * 0.5
        va_n, vb_n, vc_n = va / half_Vdc, vb / half_Vdc, vc / half_Vdc
        vmax = max(va_n, vb_n, vc_n)
        vmin = min(va_n, vb_n, vc_n)
        v_offset = -(vmax + vmin) * 0.5
        da = max(0, min(1, (va_n + v_offset + 1) * 0.5))
        db = max(0, min(1, (vb_n + v_offset + 1) * 0.5))
        dc = max(0, min(1, (vc_n + v_offset + 1) * 0.5))
        return da, db, dc

    class FOCController:
        """Stateful FOC controller (Python fallback)."""
        def __init__(self, config=None):
            self.config = config or FOCConfig()
            self._int_id = 0.0
            self._int_iq = 0.0
            self._int_speed = 0.0

        def configure(self, config):
            self.config = config
            self.reset()

        def reset(self):
            self._int_id = 0.0
            self._int_iq = 0.0
            self._int_speed = 0.0

        def step(self, ia, ib, ic, theta_e, omega_m, speed_ref, id_ref):
            cfg = self.config
            dt = cfg.Ts

            alpha, beta = clarke(ia, ib, ic)
            d_cur, q_cur = park(alpha, beta, theta_e)

            speed_error = speed_ref - omega_m
            self._int_speed += speed_error * dt
            iq_ref = cfg.Kp_speed * speed_error + cfg.Ki_speed * self._int_speed
            iq_ref = max(-cfg.iq_max, min(cfg.iq_max, iq_ref))

            id_error = id_ref - d_cur
            self._int_id += id_error * dt
            vd = cfg.Kp_id * id_error + cfg.Ki_id * self._int_id

            iq_error = iq_ref - q_cur
            self._int_iq += iq_error * dt
            vq = cfg.Kp_iq * iq_error + cfg.Ki_iq * self._int_iq

            omega_e = cfg.pole_pairs * omega_m
            vd -= omega_e * cfg.Lq * q_cur
            vq += omega_e * cfg.Ld * d_cur + omega_e * cfg.flux_pm

            Vmax = cfg.Vdc / math.sqrt(3)
            V_mag = math.sqrt(vd**2 + vq**2)
            if V_mag > Vmax:
                scale = Vmax / V_mag
                vd *= scale
                vq *= scale

            v_alpha, v_beta = inv_park(vd, vq, theta_e)
            da, db, dc = svpwm(v_alpha, v_beta, cfg.Vdc)

            out = FOCOutput()
            out.duty_a = da
            out.duty_b = db
            out.duty_c = dc
            out.id_meas = d_cur
            out.iq_meas = q_cur
            out.vd = vd
            out.vq = vq
            out.iq_ref = iq_ref
            return out
