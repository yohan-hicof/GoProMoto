//
// Created by yohan on 27.07.24.
//

#ifndef GOPROMOTO_COMPUTE_LAPS_H
#define GOPROMOTO_COMPUTE_LAPS_H

#include "extract_GPMF.h"

#ifndef M_PI
#define M_PI 3.14159265
#endif // !M_PI
#ifndef D2R
#define d2r (M_PI / 180.0)
#endif

using namespace std;

struct single_lap{
    double lap_time; //In second;
    vector<double> intermediate_time;
    vector<size_t> list_index; //Only two index if no intermediate time.
    vector<double> list_ts; //Only two index if no intermediate time. Instead of using index, use timestamp, more precise
    string s_lap_time;
    vector<string> s_intermediate_time;
};

struct laps_data{ //Init the rate to -1 to know when not init.

    string track_name;
    double gps_hz;
    double finish_lat, finish_long; //The latitude and longitude of the finish line. Used for lap computation.
    vector<double> intermediate_lat, intermediate_long; // The list of intermediate points. Can be empty.
    // If not empty, the first and last one are the finish line.

    vector<size_t> list_finish; // The indices when we crossed the finish line.
    vector<vector<size_t> > list_intermediate; // For each intermediate point, the indices when we cross them

    vector<single_lap> list_laps;

};

string time_to_string(double t);

string diff_index_to_string(size_t index1, size_t index2, double hz);

vector<size_t> find_coordinates(const extracted_data &data, double &lat, double &lon, double max_dist, size_t start_index, size_t end_index);
vector<double> find_coordinates_ts(const extracted_data &data, double &lat, double &lon, double max_dist, double start_ts, size_t end_ts);
double haversine_meters(double lat1, double long1, double lat2, double long2);

void extract_lap_info(const string filename, const extracted_data &data, laps_data &laps);

#endif //GOPROMOTO_COMPUTE_LAPS_H
