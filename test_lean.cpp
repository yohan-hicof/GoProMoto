struct FrameData {
    double accl_x, accl_y, accl_z;
    double gyro_x, gyro_y, gyro_z;
    double gps_speed;           // m/s
    double lean_angle;          // degrees, right = positive
    double longitudinal_accel;  // m/s², positive = accelerating
};

struct CalibRef {
    double offset_x;  // Mounting error: subtract from accl_x to get "true lateral"
    double offset_y;  // Mounting error: subtract from accl_y
    double g;         // Measured gravity magnitude
};


void average_accel_gyro_data(extracted_data &data, int windows_size){
    // Average the data extracted from the gps to clean the signal.
    // std::vector<double> accl_x, accl_y, accl_z gyro_x, gyro_y, gyro_z;
    // We assume all these vectors of the same size
    vector<double> ax, ay, az, gx, gy, gz;
    
    for (int i = windows_size; i < static_cast<int>(data.accl_x.size())-windows_size; i++){
    	double accl_x=0.0, accl_y=0.0, accl_z=0.0, gyro_x=0.0, gyro_y=0.0, gyro_z=0.0;
    	for (int j = i-windows_size; j < i+windows_size; j++){
    	    accl_x += data.accl_x[j];
    	    accl_y += data.accl_y[j];
    	    accl_z += data.accl_z[j];
    	    gyro_x += data.gyro_x[j];
    	    gyro_y += data.gyro_y[j];
    	    gyro_z += data.gyro_z[j];    	
    	}
    	ax.push_back(acclx/(2*windows_size));
    	ay.push_back(accly/(2*windows_size));
    	az.push_back(acclz/(2*windows_size));
    	gx.push_back(gyro_x/(2*windows_size));
    	gy.push_back(gyro_y/(2*windows_size));
    	gz.push_back(gyro_z/(2*windows_size));
    }
    //Update the original vector
    for (int i = windows_size, j=0; i < static_cast<int>(data.accl_x.size())-windows_size; i++, j++){
        data.accl_x[i] = ax[j];
        data.accl_y[i] = ay[j];
        data.accl_z[i] = az[j];
        data.gyro_x[i] = gx[j];
        data.gyro_y[i] = gy[j];
        data.gyro_z[i] = gz[j];
    }

}


double interp(const std::vector<double>& data, double data_rate, double data_start,  double t) {
    double rel = t - data_start;
    double idx_f = rel * data_rate;
    int i0 = std::max(0, (int)std::floor(idx_f));
    int i1 = std::min((int)data.size() - 1, i0 + 1);
    double frac = idx_f - i0;
    return data[i0] * (1.0 - frac) + data[i1] * frac;
}


std::vector<FrameData> computePerFrame(const extracted_data& d) {
    std::vector<FrameData> frames(d.nb_frames);
    for (uint32_t i = 0; i < d.nb_frames; i++) {
        double t = d.accl_start + (double)i / d.framerate; // absolute time of frame
        frames[i].accl_x = interp(d.accl_x, d.acclrate, d.accl_start, t);
        frames[i].accl_y = interp(d.accl_y, d.acclrate, d.accl_start, t);
        frames[i].accl_z = interp(d.accl_z, d.acclrate, d.accl_start, t);
        frames[i].gyro_x = interp(d.gyro_x, d.gyrorate, d.gyro_start, t);
        frames[i].gyro_y = interp(d.gyro_y, d.gyrorate, d.gyro_start, t);
        frames[i].gyro_z = interp(d.gyro_z, d.gyrorate, d.gyro_start, t);
        frames[i].gps_speed = interp(d.gps_speed, d.gpsrate, d.gps_start, t);
    }
    return frames;
}


CalibRef calibrate(const std::vector<FrameData>& frames,
                   double skip_start_seconds, double framerate) {
    // Skip first N seconds (bike on stand)
    int skip = (int)(skip_start_seconds * framerate);

    // Find the 95th percentile GPS speed as "fast straight" threshold
    std::vector<double> speeds;
    for (int i = skip; i < frames.size(); i++)
        speeds.push_back(frames[i].gps_speed);
    std::sort(speeds.begin(), speeds.end());
    double speed_thresh = speeds[(int)(speeds.size() * 0.90)];

    // Collect accel samples where: fast + low yaw rate (straight line)
    double sum_x = 0, sum_y = 0, count = 0;
    for (int i = skip; i < (int)frames.size(); i++) {
        bool is_fast     = frames[i].gps_speed > speed_thresh;
        bool is_straight = std::abs(frames[i].gyro_z) < 0.05; // rad/s threshold
        if (is_fast && is_straight) {
            sum_x += frames[i].accl_x;
            sum_y += frames[i].accl_y;
            count++;
        }
    }

    double mean_x = sum_x / count;
    double mean_y = sum_y / count;

    return {
        .offset_x = mean_x,                              // Should be ~0 ideally
        .offset_y = mean_y - 9.81,                       // Should be ~9.81 ideally
        .g        = std::sqrt(mean_x*mean_x + mean_y*mean_y)
    };
}


void computeLeanAngle(std::vector<FrameData>& frames,
                      const CalibRef& cal, double framerate,
                      double alpha = 0.98) {
    const double dt = 1.0 / framerate;
    const double RAD2DEG = 180.0 / M_PI;

    double lean = 0.0; // Initial lean (will converge quickly)

    for (int i = 0; i < (int)frames.size(); i++) {
        // Calibrated lateral and vertical components
        double ax = frames[i].accl_x - cal.offset_x;
        double ay = frames[i].accl_y - (cal.g + cal.offset_y); // re-center to true g

        // Accelerometer-based lean: angle of gravity in lateral/vertical plane
        double accel_lean = std::atan2(ax, ay); // radians

        // Gyroscope roll rate around the forward axis
        // For GoPro facing forward: lean rate ≈ -gyro_z
        // *** Verify sign: lean right should give positive angle ***
        double gyro_roll_rate = -frames[i].gyro_z; // rad/s

        // Complementary filter
        lean = alpha * (lean + gyro_roll_rate * dt) + (1.0 - alpha) * accel_lean;

        frames[i].lean_angle = lean * RAD2DEG;

        // Clamp absurd values (e.g. bike on stand at start)
        if (std::abs(frames[i].lean_angle) > 75.0)
            frames[i].lean_angle = std::copysign(75.0, frames[i].lean_angle);
    }
}

void computeLongitudinalAccel(std::vector<FrameData>& frames,
                               double framerate) {
    const double dt = 1.0 / framerate;

    // Central difference for interior points
    for (int i = 1; i < (int)frames.size() - 1; i++) {
        frames[i].longitudinal_accel =
            (frames[i+1].gps_speed - frames[i-1].gps_speed) / (2.0 * dt);
    }
    // Edge cases
    frames[0].longitudinal_accel = frames[1].longitudinal_accel;
    frames.back().longitudinal_accel = frames[frames.size()-2].longitudinal_accel;

    // Optional: light smoothing to reduce GPS noise
    // A simple 5-frame moving average:
    const int W = 5;
    std::vector<double> smoothed(frames.size());
    for (int i = 0; i < (int)frames.size(); i++) {
        double sum = 0; int cnt = 0;
        for (int j = i - W/2; j <= i + W/2; j++) {
            if (j >= 0 && j < (int)frames.size()) { sum += frames[j].longitudinal_accel; cnt++; }
        }
        smoothed[i] = sum / cnt;
    }
    for (int i = 0; i < (int)frames.size(); i++)
        frames[i].longitudinal_accel = smoothed[i];
}

