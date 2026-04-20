I have to find a way to compute the lean angle of the motorbike, also if possible, acceleration and brake.

So far, I have a structure with all the data extracted from the gopro video:
struct extracted_data{ //Init the rate to -1 to know when not init.
    double framerate = -1; //How many frames per second in the video.
    uint32_t nb_frames = 0; //Number of frames in the video.    
    double acclrate = -1, accl_start, accl_end;
    double gyrorate = -1, gyro_start, gyro_end;
    double gpsrate = -1, gps_start, gps_end;
    std::vector<double> gps_lat, gps_long, gps_alt, gps_speed, gps_speed2;
    string gps_lat_unit, gps_long_unit, gps_alt_unit, gps_speed_unit, gps_speed2_unit;

    std::vector<double> accl_x, accl_y, accl_z;
    string accl_x_unit, accl_y_unit, accl_z_unit;

    std::vector<double> gyro_x, gyro_y, gyro_z;
    string gyro_x_unit, gyro_y_unit, gyro_z_unit;

};

Using that, I should be able to compute the lean angle of the motorcycle, and its braking/acceleration.
I have the following assumpution:
 - The camera is aligned with the bike direction, facing forward.
 - I can that the bike is straight on the straight line, i.e. when the bike is at its fastest. I could do that once per turn.
 - The lean angle is probably lower than 60 degree, except for accident.
 - I should probably not use first few seconds of the video because the bike might be on the stand.
 - I should use whole dataset to have the best precision, then compute for each time point.
