//
// Created by yohan on 6/13/24.
//

#ifndef EXTRACT_GPMF_H
#define EXTRACT_GPMF_H

#include <iostream>
#include <vector>
#include <cmath>
#include <stdint.h>
#include <fstream>

#include <opencv2/opencv.hpp>

#include "./GPMF/GPMF_parser.h"
#include "./GPMF/GPMF_mp4reader.h"
#include "./GPMF/GPMF_utils.h"
#include "./GPMF/GPMF_bitstream.h"
#include "./GPMF/GPMF_common.h"

#include "structures.h"

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

//I redefined it here to not move everything around...
double haversine_meters(double lat1, double long1, double lat2, double long2);

int get_mp4_data(const char* filename, extracted_data &data);
void write_mp4_metadata(const string filename, extracted_data &data);
void write_mp4_all_metadata(const string filename, extracted_data &data, size_t nb_data_points=0);
void correct_gps_data(extracted_data &data);
bool check_gps_data(extracted_data &data);
//bool create_gps_object(gps_data &gps, extracted_data &data, int nb_cols, int nb_rows);

void convert_ms2kmh(extracted_data &data);

void average_imu_data(extracted_data &data, int window_size=5);

GPMF_ERR GetGPSMP4File(const char* filename, extracted_data &data);
GPMF_ERR GetACCLMP4File(const char* filename, extracted_data &data);
GPMF_ERR GetGYROMP4File(const char* filename, extracted_data &data);

void cleanGPSjumps(extracted_data &data, laps_data &laps, double maxDistFromTrackM = 300.0, double maxStepM = 50.0);

#endif //EXTRACT_GPMF_H
