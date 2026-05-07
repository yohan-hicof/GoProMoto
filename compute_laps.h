//
// Created by yohan on 27.07.24.
//

#ifndef GOPROMOTO_COMPUTE_LAPS_H
#define GOPROMOTO_COMPUTE_LAPS_H

#include "extract_GPMF.h"
#include "structures.h"

#include <vector>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <algorithm>
#include <fstream>

#ifndef M_PI
#define M_PI 3.14159265
#endif // !M_PI
#ifndef D2R
#define d2r (M_PI / 180.0)
#endif

using namespace std;



string time_to_string(double t);

string diff_index_to_string(size_t index1, size_t index2, double hz);

vector<double> find_coordinates_ts(const extracted_data &data, double &lat, double &lon, double max_dist, double start_ts, double end_ts);
double haversine_meters(double lat1, double long1, double lat2, double long2);

void extract_lap_info(const string filename, extracted_data &data, laps_data &laps);



std::vector<single_lap> findLaps(
    const std::vector<double>& lat,
    const std::vector<double>& lon,
    const std::vector<double>& ts,
    double finishLat,  double finishLon,
    const std::vector<double>& intermLat,
    const std::vector<double>& intermLon,
    double thresholdM    = 15.0,
    double minLapTimeSec = 10.0);
#endif //GOPROMOTO_COMPUTE_LAPS_H
