
#include "lean_functions.hpp"


std::vector<double> gaussianSmooth(const std::vector<double>& data, double sigma){
    const int n = static_cast<int>(data.size());
    const int hw = static_cast<int>(std::ceil(3.0 * sigma)); // half-window

    // build normalised kernel
    std::vector<double> kernel(2 * hw + 1);
    double ksum = 0.0;
    for (int k = -hw; k <= hw; ++k) {
        kernel[k + hw] = std::exp(-0.5 * k * k / (sigma * sigma));
        ksum += kernel[k + hw];
    }
    for (auto& v : kernel) v /= ksum;

    std::vector<double> out(n);
    for (int i = 0; i < n; ++i) {
        double val = 0.0;
        for (int k = -hw; k <= hw; ++k) {
            // reflect at boundaries
            int j = i + k;
            if (j < 0)     j = -j;
            if (j >= n)    j = 2 * n - 2 - j;
            val += kernel[k + hw] * data[j];
        }
        out[i] = val;
    }
    return out;
}

/// Velocity via central differences, non-uniform timestamps.
///   v_i = (p_{i+1} - p_{i-1}) / (t_{i+1} - t_{i-1})
/// Endpoints use one-sided differences.
std::vector<double> velocity(const std::vector<double>& p, const std::vector<double>& t){
    const int n = static_cast<int>(p.size());
    std::vector<double> v(n);

    // forward difference at left endpoint
    v[0] = (p[1] - p[0]) / (t[1] - t[0]);

    for (int i = 1; i < n - 1; ++i)
        v[i] = (p[i + 1] - p[i - 1]) / (t[i + 1] - t[i - 1]);

    // backward difference at right endpoint
    v[n - 1] = (p[n - 1] - p[n - 2]) / (t[n - 1] - t[n - 2]);

    return v;
}

/// Acceleration via non-uniform central differences.
///   a_i = 2/(dt_f+dt_b) * [(p_{i+1}-p_i)/dt_f - (p_i-p_{i-1})/dt_b]
/// Endpoints are extrapolated from neighbours.
std::vector<double> acceleration(const std::vector<double>& p, const std::vector<double>& t){
    const int n = static_cast<int>(p.size());
    std::vector<double> a(n);

    for (int i = 1; i < n - 1; ++i) {
        const double dt_f = t[i + 1] - t[i];
        const double dt_b = t[i]     - t[i - 1];
        a[i] = 2.0 / (dt_f + dt_b) *
               ((p[i + 1] - p[i]) / dt_f - (p[i] - p[i - 1]) / dt_b);
    }
    a[0]     = a[1];
    a[n - 1] = a[n - 2];

    return a;
}

/// Estimate lean angle for every sample.
///
/// @param lat       Latitude  vector [degrees], size N
/// @param lon       Longitude vector [degrees], size N
/// @param alt       Altitude  vector [metres],  size N
/// @param timestamp Timestamp vector [seconds], size N – must be strictly increasing
/// @param smoothSigma  Gaussian smoothing sigma in *samples*.
///                     A value around 3–8 works well for 18 Hz GoPro GPS.
///                     Set to 0 to disable smoothing (not recommended).
///
/// @return Result struct with lean angle, speed, and curvature vectors.
lean_data lean_estimator(const std::vector<double>& lat,  const std::vector<double>& lon, const std::vector<double>& alt,
                         const std::vector<double>& timestamp, double smoothSigma){	

	static constexpr double R_EARTH    = 6'371'000.0; // m  (mean radius)
	static constexpr double G          = 9.81;         // m/s²
	static constexpr double MIN_SPEED_MS = 5.0;        // 18 km/h 
	//static constexpr double DEG_TO_RAD = M_PI / 180.0;
	//static constexpr double RAD_TO_DEG = 180.0 / M_PI;
    const int n = static_cast<int>(lat.size());
    if (n < 3)
        throw std::invalid_argument("Need at least 3 samples.");
    if (static_cast<int>(lon.size()) != n ||
        static_cast<int>(alt.size()) != n ||
        static_cast<int>(timestamp.size()) != n)
        throw std::invalid_argument("All input vectors must have the same length.");

    // ── 1. Convert to local ENU (metres) ──────────────────
    //   Origin = first sample.
    //   East  = Δlon × cos(lat₀) × R
    //   North = Δlat × R
    //   Up    = Δalt          (not used for horizontal curvature, kept for ref)
    const double lat0_rad = lat[0] * d2r;

    std::vector<double> east(n), north(n);
    for (int i = 0; i < n; ++i) {
        const double dlat = (lat[i] - lat[0]) * d2r;
        const double dlon = (lon[i] - lon[0]) * d2r;
        east[i]  = dlon * std::cos(lat0_rad) * R_EARTH;
        north[i] = dlat * R_EARTH;
    }

    // ── 2. Smooth positions ────────────────────────────────
    if (smoothSigma > 0.0) {
        east  = gaussianSmooth(east,  smoothSigma);
        north = gaussianSmooth(north, smoothSigma);
    }

    // ── 3. Velocity and acceleration (2-D horizontal) ─────
    const auto vE = velocity(east,  timestamp);
    const auto vN = velocity(north, timestamp);
    const auto aE = acceleration(east,  timestamp);
    const auto aN = acceleration(north, timestamp);

    // ── 4 & 5. Curvature → lean angle ─────────────────────
    lean_data result;
    result.lean_angle.resize(n);
    result.speedKmh.resize(n);
    result.curvature.resize(n);
    result.ts.resize(n);

    for (int i = 0; i < n; ++i) {
        const double speed2 = vE[i] * vE[i] + vN[i] * vN[i]; // m²/s²
        const double speed  = std::sqrt(speed2);               // m/s

        result.speedKmh[i] = speed * 3.6;
        result.ts[i] = timestamp[i];

        if (speed < MIN_SPEED_MS) {
            // Too slow – GPS noise dominates; output 0°
            result.lean_angle[i] = 0.0;
            result.curvature[i]    = 0.0;
            continue;
        }

        // Signed curvature (right-hand rule in ENU frame).
        // κ > 0  →  turning left
        // κ < 0  →  turning right
        const double cross = vE[i] * aN[i] - vN[i] * aE[i];
        const double kappa = cross / (speed2 * speed);

        // Lateral (centripetal) acceleration [m/s²], signed
        const double a_lat = speed2 * kappa;

        // Steady-state lean angle [degrees]
        // θ = atan(a_lat / g)
        result.lean_angle[i] = std::atan2(a_lat, G) * r2d;
        result.curvature[i]    = kappa;
    }

    //Slightly smooth the lean angle, they shake to much
    result.lean_angle = gaussianSmooth(result.lean_angle, 1.5);

    return result;
}