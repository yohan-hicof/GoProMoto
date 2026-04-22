//
// Created by yohan on 26.07.24.
//

#ifndef GOPROMOTO_OVERLAY_FUNCTIONS_H
#define GOPROMOTO_OVERLAY_FUNCTIONS_H

#include <iostream>
#include <vector>
#include <fstream>
#include <chrono>
#include <sstream>
#include <deque>

#include <opencv2/opencv.hpp>
#include "images.h"

#include "extract_GPMF.h"
#include "compute_laps.h"


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

void create_speed_meter(speed_overlay &speed, const size_t width, const size_t height);
void display_speed(speed_overlay &speed, double v);

cv::Mat get_digit_image_from_h();
void fill_digit(cv::Mat &to_fill, int digit, cv::Scalar color = cv::Scalar(23,201,232));

void display_position_gps(gps_data &gps, extracted_data &data, double ts);
bool create_gps_object(gps_data &gps, extracted_data &data, int nb_cols, int nb_rows);

cv::Mat display_lap_time(laps_data &laps, size_t index);
cv::Mat display_lap_time_ts(laps_data &laps, double start_ts);


#endif //GOPROMOTO_OVERLAY_FUNCTIONS_H
