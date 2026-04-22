//
// Created by yohan on 27.07.24.
//

#include "compute_laps.h"

string time_to_string(double t){
    //Convert a time give in second in a double into a string: xx:xx:xx (minutes, seconds, milliseconds)
    string minutes, seconds, milli, result;
    int temp;
    temp = static_cast<int>(100*t)%100;
    milli = to_string(temp);
    temp = t;
    minutes = to_string(static_cast<int>(temp/60));
    seconds = to_string(temp - static_cast<int>(temp/60)*60);
    //Add leading zeros if needed
    if (milli.size() == 1) milli = milli + "0";
    if (minutes.size() == 1) minutes = "0" + minutes;
    if (seconds.size() == 1) seconds = "0" + seconds;
    result = minutes + ":" + seconds + ":" + milli;

    //cout << "From " << t << " to " << result << endl;
    return result;
}

string diff_index_to_string(size_t index1, size_t index2, double hz){
    //From two indices and a frequence, compute the current time.
    double difference = index2 - index1;
    difference /= hz;
    return time_to_string(difference);
}

vector<string> parse_line(string line){

    vector<string> result;
    size_t pos = line.find(",");
    while (pos != string::npos){
        string token = line.substr(0, pos);
        line = line.substr(pos + 1, line.length() - 1);
        result.emplace_back(token);
        //cout << token << '\n';
        pos = line.find(",");
    }
    //cout << line << endl;
    result.emplace_back(line);
    return result;
}

void extract_lap_info(const string filename, const extracted_data &data, laps_data &laps){
    //Open the given file, for each line, extract the coordinates of the finish line.
    // Compute the distance between the finish line and the first point of the data
    //If less than 5km, this is the current track.
    // Then extract the coordinates of the finish line and intermediate points.
    //Track file structure: track_name,finish line lat, finish linelong, inter 1 lat, inter 1 long, inter 2 lat, ...

    if (data.gps_lat.empty()){
        cerr << "Error in function extract lap info, no gps coordinate information" << endl;
        return;
    }
    //Get the average lat/long to find the proper track
    double ave_lat = 0.0, ave_long = 0.0, max_distance = 20.0;
    for (auto l:data.gps_lat) ave_lat += l;
    for (auto l:data.gps_long) ave_long += l;
    ave_lat /= data.gps_lat.size();
    ave_long /= data.gps_long.size();

    laps.gps_hz = data.gpsrate;

    //Extract the track information from the file.
    ifstream file(filename);
    string line;
    while (getline(file, line)){
        std::vector<string> vect = parse_line(line);
        if ((vect.size() < 3 && vect.size() > 0) || vect.size()%2 == 0){// Should not be even either
            cerr << "Warning incorrect file format: " << vect.size() << endl;
            continue;
        }
        double lat = stod(vect[1]);
        double lon = stod(vect[2]);
        double distance = haversine_meters(lat, lon, ave_lat, ave_long);
        // cerr << vect[0] << ": " << distance/1000 << "Km." << endl;
        if (distance < 5000){// Less than 5km, we found the track.
            laps.track_name=vect[0];
            laps.finish_lat=lat;
            laps.finish_long=lon;

            for (size_t i = 3; i < vect.size(); i+=2){
                laps.intermediate_lat.emplace_back(stod(vect[i]));
                laps.intermediate_long.emplace_back(stod(vect[i+1]));
            }
            break;
        }
    }

    if (!laps.track_name.empty()){
        cout << "Found track: " << laps.track_name;
        cout << ", finish line: [" << laps.finish_lat << "," << laps.finish_long << "]\n";
    }
    else{
        return;
    }

    //Compute each lap time and its intermediate values.
    vector<size_t> list_finish = find_coordinates(data, laps.finish_lat, laps.finish_long, max_distance, 0, 0);
    vector<double> list_finish_ts = find_coordinates_ts(data, laps.finish_lat, laps.finish_long, max_distance, -100, -100);
    for (size_t i = 1; i < list_finish.size(); i++){
        single_lap curr_lap;
        curr_lap.list_index.emplace_back(list_finish[i-1]);
        curr_lap.list_ts.emplace_back(list_finish_ts[i-1]);
        for (size_t j = 0; j < laps.intermediate_lat.size(); j++){
            //This vector should contain a single value
            vector<size_t> list_inter = find_coordinates(data, laps.intermediate_lat[j], laps.intermediate_long[j], max_distance, list_finish[i-1], list_finish[i]);
            vector<double> list_inter_ts = find_coordinates_ts(data, laps.intermediate_lat[j], laps.intermediate_long[j], max_distance, list_finish_ts[i-1], list_finish_ts[i]);
            if (!list_inter.empty()) curr_lap.list_index.emplace_back(list_inter[0]);
            if (!list_inter_ts.empty()) curr_lap.list_ts.emplace_back(list_inter_ts[0]);
        }
        curr_lap.list_index.emplace_back(list_finish[i]);
        curr_lap.list_ts.emplace_back(list_finish_ts[i]);
        //Now compute the lap time and intermediate times.
        //curr_lap.lap_time = curr_lap.list_index.back() - curr_lap.list_index[0];
        //curr_lap.lap_time /= laps.gps_hz;
        curr_lap.lap_time = curr_lap.list_ts.back() - curr_lap.list_ts[0];
        for (size_t j = 1; j < curr_lap.list_index.size(); j++){
            //double t = curr_lap.list_index[j] - curr_lap.list_index[j-1];
            //t /= laps.gps_hz;
            //curr_lap.intermediate_time.emplace_back(t);
            curr_lap.intermediate_time.emplace_back(curr_lap.list_ts[j] - curr_lap.list_ts[j-1]);
        }
        //Generate the strings to be displayed
        curr_lap.s_lap_time = time_to_string(curr_lap.lap_time);
        for (auto t: curr_lap.intermediate_time) curr_lap.s_intermediate_time.emplace_back(time_to_string(t));

        laps.list_laps.emplace_back(curr_lap);
    }

    for (auto lap:laps.list_laps){
        cout << "Lap time: " << lap.s_lap_time << " -> ";
        for (auto t: lap.intermediate_time){
            cout << "|" << t;
        }
        cout << "|\n";
    }
}


double haversine_meters(double lat1, double long1, double lat2, double long2){
    //TODO, considering the distances, this function is an overkill
    //Compute the distance in meters between two gps coordinates
    double dlong = (long2 - long1) * d2r;
    double dlat = (lat2 - lat1) * d2r;
    double a = pow(sin(dlat/2.0), 2) + cos(lat1*d2r) * cos(lat2*d2r) * pow(sin(dlong/2.0), 2);
    double c = 2 * atan2(sqrt(a), sqrt(1-a));
    double d = 6367 * 1000 * c;
    return d;
}

vector<size_t>
find_coordinates(const extracted_data &data, double &lat, double &lon, double max_dist, size_t start_index,
                 size_t end_index) {
    //Find the list of closest points to the given coordinates. The point have to be closer than max_dist (in meters)
    //We keep only local minimal. When we find one, the next one should be next lap.
    // When we cross, we increase the index by 100 (a bit more than 5 seconds, assuming 18pps, or 12s for 8pps)

    vector<size_t> result;
    double min_dist = max_dist;
    size_t min_index = 0;
    if (end_index == 0) end_index = data.gps_lat.size();

    for (size_t i = start_index; i < end_index; i++){
        double distance = haversine_meters(lat, lon, data.gps_lat[i], data.gps_long[i]);
        if (distance > 20*max_dist) {i+=5; continue;}
        if (distance > max_dist) continue;

        if (distance < min_dist){//getting closer to the line;
            min_dist = distance;
            min_index = i;
        }
        else{// Getting away from the line.
            result.emplace_back(min_index);
            min_dist = max_dist;
            i+= 100;
        }
    }
    return result;
}

vector<double> find_coordinates_ts(const extracted_data &data, double &lat, double &lon, double max_dist, double start_ts, size_t end_ts) {
    //Find the list of closest points to the given coordinates. The point have to be closer than max_dist (in meters)
    // We keep only local minimal. When we find one, the next one should be next lap.
    // When we cross, we increase the ts by 5s to go to the next lap

    vector<double> result;
    double min_dist = max_dist, min_ts;
    //Update the gps timestamp if needed
    if (start_ts < -10) start_ts = data.gps_ts[0];
    if (end_ts < -10) end_ts = data.gps_ts[data.gps_ts.size()-1];


    for (size_t i = 0; i < data.gps_lat.size(); i++){
        // Ensure we are in the proper range
        if (data.gps_ts[i] < start_ts || data.gps_ts[i] > end_ts) continue;
        double distance = haversine_meters(lat, lon, data.gps_lat[i], data.gps_long[i]);
        if (distance > 20*max_dist) {i+=5; continue;}
        if (distance > max_dist) continue;

        if (distance < min_dist){//getting closer to the line;
            min_dist = distance;
            min_ts = data.gps_ts[i];
        }
        else{// Getting away from the line.
            result.emplace_back(min_ts);
            min_dist = max_dist;
            i+= 100;
        }
    }
    return result;
}
