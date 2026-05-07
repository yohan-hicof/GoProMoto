#ifndef GOPROMOTO_STRUCTURES_H
#define GOPROMOTO_STRUCTURES_H

#include <iostream>
#include <cstdint>
#include <vector>

#include <opencv2/opencv.hpp>

using namespace std;


struct extracted_data{ //Init the rate to -1 to know when not init.
    double framerate = -1; //How many frames per second in the video.
    uint32_t nb_frames = 0; //Number of frames in the video.
    //For acceleration, gyroscope and GPS: How many sample per secoond,
    //when does the samples star and end in the video, in frame (i.e. gps might not be at the beginning of the video.)
    double acclrate = -1, accl_start, accl_end;
    double gyrorate = -1, gyro_start, gyro_end;
    double gpsrate = -1, gps_start, gps_end;
    std::vector<double> gps_lat, gps_long, gps_alt, gps_speed, gps_speed2, gps_ts;
    string gps_lat_unit, gps_long_unit, gps_alt_unit, gps_speed_unit, gps_speed2_unit;

    std::vector<double> accl_x, accl_y, accl_z;
    string accl_x_unit, accl_y_unit, accl_z_unit;

    std::vector<double> gyro_x, gyro_y, gyro_z;
    string gyro_x_unit, gyro_y_unit, gyro_z_unit;

};

struct gps_data{
    //This structure contains all the data to create the overlay map without having to recompute
    //them each time we want to update the rider position.
    string track_name;
    int cols, rows;
    int position_circle;
    int road_circle;
    cv::Mat blank_map; //Reference image with just the track.
    cv::Mat position_map; //Map with the position overlayed
    bool rotate_image=false;
    double gpsrate = -1, gps_start, gps_end;

    double min_lat = DBL_MAX, max_lat = DBL_MIN; //The min/max values of the lat and long
    double min_long = DBL_MAX, max_long = DBL_MIN;
    double delta_lat, delta_long; //The delta of the values, used to scale
    double ratio_wh, ratio_longlat, scaling; //The scaling used to convert from lat/long to x,y
    vector<cv::Point> list_points; //We need to keep that for when we merge several files.
    int x_shift, y_shift; //Since we resize the image and add border, we need that.
};

struct lean_data {
    //Contain the computed pitch and roll value for each frames.
    vector<double> pitch;
    vector<double> roll;
};

struct single_lap{
    double lap_time; //In second;
    vector<double> intermediate_time;
    vector<size_t> list_index; //Only two index if no intermediate time.
    vector<double> list_ts; //Only two index if no intermediate time. Instead of using index, use timestamp, more precise
    string s_lap_time;
    vector<string> s_intermediate_time;
};

struct laps_data{ 

    string track_name;
    double gps_hz;
    double finish_lat=-1000, finish_long=-1000; //The latitude and longitude of the finish line. Used for lap computation.
    vector<double> intermediate_lat, intermediate_long; // The list of intermediate points. Can be empty.
    // If not empty, the first and last one are the finish line.

    vector<size_t> list_finish; // The indices when we crossed the finish line.
    vector<vector<size_t> > list_intermediate; // For each intermediate point, the indices when we cross them

    vector<single_lap> list_laps;

};

struct global_struct{
    vector<string> paths_in;
    string path_out;
    bool overlay_track = true;
    string path_tracks_info;
    bool auto_crop = false;
    double crop_start_ts = 0.0;
    double crop_end_ts = DBL_MAX;
    double crop_shift_ts = 5.0;
};

#endif