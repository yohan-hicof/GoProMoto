

#ifndef GOPROMOTO_OVERLAY_CAIRO_FUNCTIONS_H
#define GOPROMOTO_OVERLAY_CAIRO_FUNCTIONS_H

#include <iostream>
#include <vector>
#include <fstream>
#include <chrono>
#include <sstream>
#include <deque>

#include <cairo/cairo.h>
#include <opencv2/opencv.hpp>
#include "structures.h"
#include "compute_laps.h"


//Helper functions
cairo_surface_t* mat_to_cairo(const cv::Mat& mat);
cv::Mat cairo_to_mat(cairo_surface_t* surface);
static void rounded_rect(cairo_t* cr, double x, double y, double w, double h, double r);
static void tick(cairo_t* cr, double cx, double cy, double r_outer, double r_inner, double theta_deg);
static void centred_text(cairo_t* cr, double x, double y, const std::string& s);
void overlay_hud(cv::Mat& frame, const cv::Mat& hud_bgra);
void update_HUD_image_size(int width, int height);

//Functions to call once.
cv::Mat create_static_hud(int frame_w, int frame_h);
void create_speed_hud(cv::Mat &canvas);
void create_lean_hud(cv::Mat &canvas);
void create_track_hud(cv::Mat &canvas, extracted_data &data, gps_data &gps, laps_data &ld);

//Functions to call for each frame
cv::Mat draw_speed_hud(const cv::Mat& static_layer, double speed_kmh);
cv::Mat draw_lean_hud(const cv::Mat& static_layer, double lean_deg);
cv::Mat draw_track_hud(cv::Mat &static_layer, gps_data &gps, double ts);
cv::Mat draw_track_hud_v2(cv::Mat &static_layer, gps_data &gps, laps_data &laps, double ts);
cv::Mat draw_laptime_hud(cv::Mat &static_layer, laps_data &laps, double start_ts);

static inline double deg2rad(double d) { return d * M_PI / 180.0; }

#endif //GOPROMOTO_OVERLAY_CAIRO_FUNCTIONS_H
