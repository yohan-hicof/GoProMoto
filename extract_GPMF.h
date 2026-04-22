//
// Created by yohan on 6/13/24.
//

#ifndef EXTRACT_GPMF_H
#define EXTRACT_GPMF_H

#include <iostream>
#include <vector>
#include <stdint.h>
#include <fstream>

#include <opencv2/opencv.hpp>

#include "./GPMF/GPMF_parser.h"
#include "./GPMF/GPMF_mp4reader.h"
#include "./GPMF/GPMF_utils.h"
#include "./GPMF/GPMF_bitstream.h"
#include "./GPMF/GPMF_common.h"


#define	SHOW_VIDEO_FRAMERATE		1
#define	SHOW_PAYLOAD_TIME			1
#define	SHOW_ALL_PAYLOADS			0
#define SHOW_GPMF_STRUCTURE			0
#define	SHOW_PAYLOAD_INDEX			0
#define	SHOW_SCALED_DATA			1
#define	SHOW_THIS_FOUR_CC			STR2FOURCC("ACCL")
//#define	SHOW_THIS_FOUR_CC			STR2FOURCC("GPS5")
#define SHOW_COMPUTED_SAMPLERATES	1

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

//I redefined it here to not move everything around...
double haversine_meters(double lat1, double long1, double lat2, double long2);

int get_mp4_data(const char* filename, extracted_data &data);
void write_mp4_metadata(const string filename, extracted_data &data);
void write_mp4_all_metadata(const string filename, extracted_data &data, size_t nb_data_points=0);
void correct_gps_data(extracted_data &data);
bool create_gps_object(gps_data &gps, extracted_data &data, int nb_cols, int nb_rows);

void convert_ms2kmh(extracted_data &data);

void average_imu_data(extracted_data &data, int window_size=5);

GPMF_ERR GetGPSMP4File(const char* filename, extracted_data &data);
GPMF_ERR GetACCLMP4File(const char* filename, extracted_data &data);
GPMF_ERR GetGYROMP4File(const char* filename, extracted_data &data);

#endif //EXTRACT_GPMF_H
