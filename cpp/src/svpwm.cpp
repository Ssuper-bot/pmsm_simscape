#include "svpwm.h"
#include <algorithm>
#include <cmath>

namespace pmsm {

DutyCycles svpwm(double v_alpha, double v_beta, double Vdc) {
    constexpr double sqrt3_2 = 0.86602540378;

    // Inverse Clarke to get three-phase voltages
    double va = v_alpha;
    double vb = -0.5 * v_alpha + sqrt3_2 * v_beta;
    double vc = -0.5 * v_alpha - sqrt3_2 * v_beta;

    // Normalize by Vdc/2
    double half_Vdc = Vdc * 0.5;
    double va_n = va / half_Vdc;
    double vb_n = vb / half_Vdc;
    double vc_n = vc / half_Vdc;

    // Min-max injection (centered SVPWM)
    double vmax = std::max({va_n, vb_n, vc_n});
    double vmin = std::min({va_n, vb_n, vc_n});
    double v_offset = -(vmax + vmin) * 0.5;

    // Convert to duty cycle [0, 1]
    DutyCycles result;
    result.da = std::clamp((va_n + v_offset + 1.0) * 0.5, 0.0, 1.0);
    result.db = std::clamp((vb_n + v_offset + 1.0) * 0.5, 0.0, 1.0);
    result.dc = std::clamp((vc_n + v_offset + 1.0) * 0.5, 0.0, 1.0);

    return result;
}

}  // namespace pmsm
