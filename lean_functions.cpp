
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
//lean_data lean_estimator(const std::vector<double>& lat,  const std::vector<double>& lon, const std::vector<double>& alt,
//                         const std::vector<double>& timestamp, double smoothSigma){	

lean_data lean_estimator(const extracted_data& data, double smoothSigma){


	static constexpr double R_EARTH    = 6'371'000.0; // m  (mean radius)
	static constexpr double G          = 9.81;         // m/s²
	static constexpr double MIN_SPEED_MS = 5.0;        // 18 km/h 
	//static constexpr double DEG_TO_RAD = M_PI / 180.0;
	//static constexpr double RAD_TO_DEG = 180.0 / M_PI;
    const int n = static_cast<int>(data.gps_lat.size());
    if (n < 3)
        throw std::invalid_argument("Need at least 3 samples.");
    
    // ── 1. Convert to local ENU (metres) ──────────────────
    //   Origin = first sample.
    //   East  = Δlon × cos(lat₀) × R
    //   North = Δlat × R
    //   Up    = Δalt          (not used for horizontal curvature, kept for ref)
    const double lat0_rad = data.gps_lat[0] * d2r;

    std::vector<double> east(n), north(n);
    for (int i = 0; i < n; ++i) {
        const double dlat = (data.gps_lat[i] - data.gps_lat[0]) * d2r;
        const double dlon = (data.gps_long[i] - data.gps_long[0]) * d2r;
        east[i]  = dlon * std::cos(lat0_rad) * R_EARTH;
        north[i] = dlat * R_EARTH;
    }

    // ── 2. Smooth positions ────────────────────────────────
    if (smoothSigma > 0.0) {
        east  = gaussianSmooth(east,  smoothSigma);
        north = gaussianSmooth(north, smoothSigma);
    }

    // ── 3. Velocity and acceleration (2-D horizontal) ─────
    const auto vE = velocity(east,  data.gps_ts);
    const auto vN = velocity(north, data.gps_ts);
    const auto aE = acceleration(east,  data.gps_ts);
    const auto aN = acceleration(north, data.gps_ts);

    // ── 4 & 5. Curvature → lean angle ─────────────────────
    lean_data result;
    result.lean_angle.resize(n);
    result.speedKmh.resize(n);
    result.curvature.resize(n);
    result.acceleration.resize(n);
    result.ts.resize(n);

    for (int i = 0; i < n; ++i) {
        const double speed2 = vE[i] * vE[i] + vN[i] * vN[i]; // m²/s²
        const double speed  = std::sqrt(speed2);               // m/s

        result.speedKmh[i] = speed * 3.6;
        result.ts[i] = data.gps_ts[i];

        if (speed < MIN_SPEED_MS) {
            // Too slow – GPS noise dominates; output 0°
            result.lean_angle[i] = 0.0;
            result.curvature[i]  = 0.0;
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

    for (std::size_t i = 5; i < n - 5; ++i) {
        //dt in s, speed in m/s -> 
        const double dt = data.gps_ts[i + 5] - data.gps_ts[i - 5];
        result.acceleration[i] = (data.gps_speed[i + 5] - data.gps_speed[i - 5]) / dt;
    }
    result.acceleration[0]   = result.acceleration[1];
    result.acceleration[n-1] = result.acceleration[n-2];


    //Slightly smooth the lean angle, they shake to much
    result.lean_angle = gaussianSmooth(result.lean_angle, 1.5);

    return result;
}




// ─────────────────────────────────────────────
//  Data types
// ─────────────────────────────────────────────

struct GpsTrack {
    std::vector<double> lat;        // decimal degrees
    std::vector<double> lon;        // decimal degrees
    std::vector<double> alt;        // meters
    std::vector<double> timestamp;  // seconds (unix or relative)
};

struct MotionEstimates {
    std::vector<double> speed;         // m/s
    std::vector<double> acceleration;  // m/s²  (+ = accel, − = braking)
    std::vector<double> lean_angle;    // degrees from vertical (0 = upright)
    std::vector<double> heading;       // degrees clockwise from North
    std::vector<double> turn_radius;   // meters (very large = straight line)
};

// ─────────────────────────────────────────────
//  Constants
// ─────────────────────────────────────────────

static constexpr double EARTH_R  = 6'371'000.0;   // mean Earth radius, metres
static constexpr double G        = 9.80665;        // standard gravity, m/s²
static constexpr double DEG2RAD  = M_PI / 180.0;
static constexpr double RAD2DEG  = 180.0 / M_PI;
static constexpr double INF_R    = 1e9;            // "straight line" sentinel

// ─────────────────────────────────────────────
//  Geometry helpers
// ─────────────────────────────────────────────

/// Great-circle distance between two WGS-84 points (metres).
inline double haversine(double lat1, double lon1,
                        double lat2, double lon2) {
    const double dLat = (lat2 - lat1) * DEG2RAD;
    const double dLon = (lon2 - lon1) * DEG2RAD;
    const double a = std::sin(dLat * 0.5) * std::sin(dLat * 0.5)
                   + std::cos(lat1 * DEG2RAD) * std::cos(lat2 * DEG2RAD)
                   * std::sin(dLon * 0.5) * std::sin(dLon * 0.5);
    return 2.0 * EARTH_R * std::asin(std::sqrt(a));
}

/// Forward bearing from point 1 → point 2, degrees [0, 360).
inline double bearing(double lat1, double lon1,
                      double lat2, double lon2) {
    const double φ1   = lat1 * DEG2RAD, φ2 = lat2 * DEG2RAD;
    const double dLon = (lon2 - lon1) * DEG2RAD;
    const double y    = std::sin(dLon) * std::cos(φ2);
    const double x    = std::cos(φ1) * std::sin(φ2)
                      - std::sin(φ1) * std::cos(φ2) * std::cos(dLon);
    const double b    = std::atan2(y, x) * RAD2DEG;
    return std::fmod(b + 360.0, 360.0);
}

/// Wrap an angle difference into (−180, +180].
inline double wrap180(double deg) {
    deg = std::fmod(deg + 180.0, 360.0);
    if (deg < 0.0) deg += 360.0;
    return deg - 180.0;
}

// ─────────────────────────────────────────────
//  Smoothing (simple centred moving average)
// ─────────────────────────────────────────────

/// Applies a centred moving-average of half-width `hw` to `v`.
/// At the edges, the window is truncated (still centred as much as possible).
inline std::vector<double> smooth(const std::vector<double>& v, int hw) {
    const int n = static_cast<int>(v.size());
    std::vector<double> out(n);
    for (int i = 0; i < n; ++i) {
        const int lo = std::max(0, i - hw);
        const int hi = std::min(n - 1, i + hw);
        double sum = 0.0;
        for (int j = lo; j <= hi; ++j) sum += v[j];
        out[i] = sum / static_cast<double>(hi - lo + 1);
    }
    return out;
}

//TODO, take the lean estimation from the other function, looked more stable...

// ─────────────────────────────────────────────
//  Core estimation
// ─────────────────────────────────────────────

/**
 * Estimate speed, acceleration, lean angle and heading from a GPS track.
 *
 * Algorithm overview
 * ──────────────────
 *  1.  Per-segment speed  v_i  = Δd_i / Δt_i          (haversine distance)
 *  2.  Per-segment heading ψ_i = bearing(p_i, p_{i+1}) 
 *  3.  Assign segment quantities to their midpoint sample,
 *      then interpolate to the full sample grid.
 *  4.  Yaw rate ψ̇  = Δψ / Δt  (with angle wrapping)
 *  5.  Lean angle θ = atan( v · ψ̇ / g )               [physics: tan θ = v²/(r·g)]
 *  6.  Turn radius r = v / |ψ̇|
 *  7.  Acceleration a = Δv / Δt  (centred finite differences)
 *
 * @param track     Input GPS data (all vectors must be the same length ≥ 3)
 * @param smooth_hw Half-width of the centred moving-average smoothing window.
 *                  Increase if your GPS is noisy (typical: 2–5 for 1 Hz data,
 *                  8–15 for 10 Hz data).
 * @return          MotionEstimates with one value per GPS sample.
 */
lean_data estimate_motion(const extracted_data& data, int smooth_hw) {
    const std::size_t N = data.gps_lat.size();
    if (N != data.gps_long.size() || N != data.gps_alt.size() || N != data.gps_ts.size())
        throw std::invalid_argument("All track vectors must have equal length.");
    if (N < 3)
        throw std::invalid_argument("Need at least 3 GPS samples.");

    // ── Step 1 & 2: per-segment speed and heading (N-1 segments) ──────────
    std::vector<double> seg_speed(N - 1), seg_heading(N - 1);

    for (std::size_t i = 0; i < N - 1; ++i) {
        const double dt   = data.gps_ts[i + 1] - data.gps_ts[i];
        if (dt <= 0.0) throw std::runtime_error("Timestamps must be strictly increasing.");

        const double dist = haversine(data.gps_lat[i],  data.gps_long[i],
                                      data.gps_lat[i+1], data.gps_long[i+1]);
        seg_speed[i]   = dist / dt;
        seg_heading[i] = bearing(data.gps_lat[i],  data.gps_long[i],
                                 data.gps_lat[i+1], data.gps_long[i+1]);
    }

    // ── Step 3: map segment quantities to sample points ───────────────────
    //   We assign each segment's value to both its endpoints and average when
    //   two segments share an endpoint. This gives a smooth, no-look-ahead result.
    std::vector<double> spd(N, 0.0), hdg(N, 0.0);
    std::vector<int>    cnt(N, 0);

    for (std::size_t i = 0; i < N - 1; ++i) {
        for (std::size_t e : {i, i + 1}) {
            spd[e] += seg_speed[i];
            // Heading: wrap-safe accumulation via Cartesian mean
            hdg[e] += std::sin(seg_heading[i] * DEG2RAD);
            // (we'll handle the cos part below)
            cnt[e]++;
        }
    }
    // Redo heading with proper circular mean (store sin and cos separately)
    std::vector<double> hsin(N, 0.0), hcos(N, 0.0);
    for (std::size_t i = 0; i < N - 1; ++i) {
        const double rad = seg_heading[i] * DEG2RAD;
        hsin[i]     += std::sin(rad);  hcos[i]     += std::cos(rad);
        hsin[i + 1] += std::sin(rad);  hcos[i + 1] += std::cos(rad);
    }
    std::vector<double> heading_raw(N);
    for (std::size_t i = 0; i < N; ++i) {
        spd[i]         /= cnt[i];
        heading_raw[i]  = std::fmod(std::atan2(hsin[i], hcos[i]) * RAD2DEG + 360.0, 360.0);
    }

    // ── Step 4: smooth speed and heading before differentiation ───────────
    const std::vector<double> spd_s = smooth(spd, smooth_hw);
    const std::vector<double> hdg_s = smooth(heading_raw, smooth_hw);
    //   Note: smoothing heading directly works because we use wrap180() below.

    // ── Step 5: yaw rate, lean angle, turn radius ─────────────────────────
    std::vector<double> lean(N, 0.0), radius(N, INF_R);

    for (std::size_t i = 1; i < N - 1; ++i) {
        const double dt = data.gps_ts[i + 1] - data.gps_ts[i - 1];
        if (dt <= 0.0) continue;

        // Centred heading difference (wrap into −180…+180 to handle 359→1)
        const double dPsi_deg = wrap180(hdg_s[i + 1] - hdg_s[i - 1]);
        const double yaw_rate = dPsi_deg * DEG2RAD / dt;   // rad/s

        const double v = spd_s[i];

        // tan(θ) = v · ψ̇ / g   →  θ = atan( v · ψ̇ / g )
        lean[i] = std::atan(v * yaw_rate / G) * RAD2DEG;   // degrees

        // r = v / |ψ̇|, guard division by zero
        radius[i] = (std::abs(yaw_rate) > 1e-6) ? (v / std::abs(yaw_rate)) : INF_R;
    }
    // Fill edges by replication
    lean[0]     = lean[1];       lean[N-1]     = lean[N-2];
    radius[0]   = radius[1];     radius[N-1]   = radius[N-2];

    // ── Step 6: acceleration (centred finite differences on smoothed speed) ─
    std::vector<double> accel(N, 0.0);
    for (std::size_t i = 1; i < N - 1; ++i) {
        const double dt = data.gps_ts[i + 1] - data.gps_ts[i - 1];
        accel[i] = (spd_s[i + 1] - spd_s[i - 1]) / dt;
    }
    accel[0]   = accel[1];
    accel[N-1] = accel[N-2];

    // ── Assemble and return ───────────────────────────────────────────────
    lean_data result;    
    result.speedKmh        = smooth(spd_s,  smooth_hw);
    result.heading      = hdg_s;
    result.acceleration = smooth(accel, smooth_hw);
    result.lean_angle   = smooth(lean,  smooth_hw);
    result.curvature  = radius;
    result.ts = data.gps_ts;
    return result;
}