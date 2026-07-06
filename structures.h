#ifndef GOPROMOTO_STRUCTURES_H
#define GOPROMOTO_STRUCTURES_H

#include <iostream>
#include <cstdint>
#include <vector>

#include <opencv2/opencv.hpp>

using namespace std;


#ifndef M_PI
#define M_PI 3.14159265

#endif // !M_PI
#define d2r (M_PI / 180.0)
#define r2d (180.0 / M_PI)

struct speed_overlay{
    //This structure contains all the data to create the overlay map without having to recompute
    //them each time we want to update the rider position.
    int cols, rows;
    cv::Mat blank_speed; //Reference image with just the track.
    cv::Mat curr_speed; //Map with the position overlayed

    cv::Point centre;
    int radius = -1;
    int thickness = -1;

    vector<cv::Mat> list_digits;

};

struct extracted_data{ //Init the rate to -1 to know when not init.
    double framerate = -1; //How many frames per second in the video.
    uint32_t nb_frames = 0; //Number of frames in the video.
    string start_ts; //The begining of the GPS data, maybe to add to the video to know when was the video taken. Or for the output file
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
    vector<cv::Point2d> track_pts; //All the points of the tracks
    vector<double> track_ts; // The time stamp corresponding to each point.   
    cv::Point2d track_finish_line;
    vector<cv::Point2d> track_intermediates;
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

struct lean_data{
    vector<double> lean_angle;
    vector<double> acceleration;
    std::vector<double> heading;
    vector<double> curvature;
    vector<double> speedKmh;
    vector<double> ts;
    cv::Mat blank_lean;   //Base image where we overlay the real position
    cv::Mat current_lean; //Final image to be overlay over the video
};

struct global_struct{
    vector<string> paths_in;
    string path_output;
    string temp_video = "./temp_video.MP4";
    string temp_audio = "./temp_audio.aac";
    bool overlay_track = true;
    string path_tracks;
    bool time_only = false; //If true, do not process the video, just display the lap times.
    bool auto_crop = false; // Automatically crop around the beginning of the first lap and end of the last one.
    bool best_lap = false; // Only keep the best lap
    double crop_start_ts = 0.0;
    double crop_end_ts = 0.0;
    double crop_shift_ts = 5.0;
    uint32_t limit_frame = INT_MAX;

    //Allow to send back the data to the main function.
    extracted_data data;
    gps_data gps;
    laps_data laps;

};

#endif