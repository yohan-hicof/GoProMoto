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
    if (milli.size() == 1) milli = "0" + milli;
    //else if (milli.size() == 2) milli = "0" + milli;
    if (minutes.size() == 1) minutes = "0" + minutes;
    if (seconds.size() == 1) seconds = "0" + seconds;
    result = minutes + ":" + seconds + ":" + milli;

    //cout << "From " << t << " to " << result << " (milli " << milli << ")" << endl;
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

bool check_lap_valid(single_lap &lap){

    //Check if a lap is valid, i.e. its time is positive, and time between intermediate is > 1s
    if (lap.lap_time < 0) return false;
    for (auto t: lap.intermediate_time)
       if (t < 1) return false;
    return true;
}

void extract_lap_info(const string filename, extracted_data &data, laps_data &laps){
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
    double ave_lat = 0.0, ave_long = 0.0, max_distance = 25.0;
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

    //Now that we know the track location, we can correct the gps data, then compute the laps
    //Check if there is an issue with the GPS data and clean them
    cleanGPSjumps(data, laps);

#if 0

    //Compute each lap time and its intermediate values.    
    vector<double> list_finish_ts = find_coordinates_ts(data, laps.finish_lat, laps.finish_long, max_distance, -100, -100);    
    cerr << "Number of finish line cross: " << list_finish_ts.size() << endl;
    
    for (size_t i = 1; i < list_finish_ts.size(); i++){
        single_lap curr_lap;        
        curr_lap.list_ts.emplace_back(list_finish_ts[i-1]);
        cerr << "Current lap: " << list_finish_ts[i-1] << " -> " << list_finish_ts[i] << endl;
        for (size_t j = 0; j < laps.intermediate_lat.size(); j++){            
            //This vector should contain a single value            
            vector<double> list_inter_ts = find_coordinates_ts(data, laps.intermediate_lat[j], laps.intermediate_long[j], max_distance, list_finish_ts[i-1], list_finish_ts[i]);                        
            if (!list_inter_ts.empty()) curr_lap.list_ts.emplace_back(list_inter_ts[0]);
        }
        curr_lap.list_ts.emplace_back(list_finish_ts[i]);
        //Now compute the lap time and intermediate times.        
        curr_lap.lap_time = curr_lap.list_ts.back() - curr_lap.list_ts[0];
        for (size_t j = 1; j < curr_lap.list_ts.size(); j++){            
            curr_lap.intermediate_time.emplace_back(curr_lap.list_ts[j] - curr_lap.list_ts[j-1]);
            cerr << "--- Intermediate: " << curr_lap.list_ts[j] - curr_lap.list_ts[j-1] << endl;
        }
        //Generate the strings to be displayed
        curr_lap.s_lap_time = time_to_string(curr_lap.lap_time);
        for (auto t: curr_lap.intermediate_time) curr_lap.s_intermediate_time.emplace_back(time_to_string(t));

        if (check_lap_valid(curr_lap))
            laps.list_laps.emplace_back(curr_lap);
        else
            cerr << "Warning, invalid lap: " << curr_lap.s_lap_time << endl;
    }
#endif

    //cerr << "---------------\nCalling claude function\n--------------------" << endl;
    //laps.list_laps.clear();
    laps.list_laps = findLaps(data.gps_lat, data.gps_long, data.gps_ts, laps.finish_lat, laps.finish_long, 
                              laps.intermediate_lat, laps.intermediate_long, 25.0, 30.0);

    for (auto lap:laps.list_laps){
        cout << "Lap time: " << lap.s_lap_time << " -> ";
        for (auto t: lap.intermediate_time){
            cout << "|" << t;
        }
        cout << "|\n";
    }

    //exit(0);

}


double haversine_meters(double lat1, double long1, double lat2, double long2){
    //TODO, considering the distances, this function is an overkill
    //Compute the distance in meters between two gps coordinates
    double dlong = (long2 - long1) * d2r;
    double dlat = (lat2 - lat1) * d2r;
    double a = pow(sin(dlat/2.0), 2) + cos(lat1*d2r) * cos(lat2*d2r) * pow(sin(dlong/2.0), 2);
    double c = 2 * atan2(sqrt(a), sqrt(1-a));
    double d = 6371 * 1000 * c;
    return d;
}


vector<double> find_coordinates_ts(const extracted_data &data, double &lat, double &lon, double max_dist, double start_ts, double end_ts) {
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
        if (data.gps_ts[i] < start_ts) continue;
        if (data.gps_ts[i] > end_ts) break;
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




///////------------------------CLAUDE------------------////////////
// ─────────────────────────────────────────────────────────────────────────────
//  Geometry helpers
// ─────────────────────────────────────────────────────────────────────────────

namespace detail {

// ─────────────────────────────────────────────────────────────────────────────
//  Crossing detector for a single checkpoint
// ─────────────────────────────────────────────────────────────────────────────

/// Returns a chronologically sorted list of timestamps at which the GPS track
/// passed through a circle of radius `thresholdM` metres centred on
/// (chkLat, chkLon).
///
/// Algorithm:
///   1. Slide through samples; maintain an "in-zone" flag.
///   2. While inside the zone, track the sample with the smallest distance.
///   3. On zone exit, apply a parabolic fit to the three samples around the
///      minimum to obtain a sub-sample precise timestamp.
///
/// The zone approach naturally handles GPS drift: a single physical crossing
/// produces exactly one event even if several consecutive samples fall inside.
///
/// @param minDwellGap  Minimum seconds between two consecutive crossings of
///                     the same checkpoint (debounce for GPS wobble near the
///                     checkpoint). Set to 0 to disable.
inline std::vector<double> findCrossings(
    const std::vector<double>& lat,
    const std::vector<double>& lon,
    const std::vector<double>& ts,
    double chkLat, double chkLon,
    double thresholdM,
    double minDwellGap = 0.0)
{
    const std::size_t N = lat.size();
    std::vector<double> crossings;

    bool        inZone  = false;
    double      minDist = std::numeric_limits<double>::max();
    std::size_t minIdx  = 0;

    auto commitCrossing = [&]()
    {
        // ── Sub-sample interpolation via parabolic fit ──────────────────────
        // We have three consecutive distance samples around the minimum:
        //   d(-1) = d_prev,  d(0) = d_min,  d(+1) = d_next
        // Fit d(t) = A·t² + B·t + C.  Minimum at t* = -B/(2A).
        // t* is in units of one sample interval; we convert to seconds using
        // the neighbouring dt values.

        double preciseTs = ts[minIdx]; // default: the raw minimum sample

        if (minIdx > 0 && minIdx + 1 < N)
        {
            const double d_prev = haversine_meters(lat[minIdx - 1], lon[minIdx - 1], chkLat, chkLon);
            const double d_min  = minDist;
            const double d_next = haversine_meters(lat[minIdx + 1], lon[minIdx + 1], chkLat, chkLon);

            const double denom = d_next - 2.0 * d_min + d_prev; // 2A
            if (std::abs(denom) > 1e-12)
            {
                double t_star = -0.5 * (d_next - d_prev) / denom; // in sample units
                t_star = std::max(-1.0, std::min(1.0, t_star));    // clamp to [-1, 1]

                if (t_star < 0.0)
                    preciseTs = ts[minIdx] + t_star * (ts[minIdx] - ts[minIdx - 1]);
                else
                    preciseTs = ts[minIdx] + t_star * (ts[minIdx + 1] - ts[minIdx]);
            }
        }

        // ── Debounce: discard if too close to the previous crossing ─────────
        if (!crossings.empty() &&
            preciseTs - crossings.back() < minDwellGap)
        {
            // Keep whichever has the smaller absolute distance
            // (prefer the "tighter" crossing for accuracy)
            if (minDist < haversine_meters(lat[minIdx], lon[minIdx], chkLat, chkLon))
                crossings.back() = preciseTs; // replace with better one
            return;
        }

        crossings.push_back(preciseTs);
    };

    for (std::size_t i = 0; i < N; ++i)
    {
        const double d = haversine_meters(lat[i], lon[i], chkLat, chkLon);

        if (d <= thresholdM)
        {
            if (!inZone)          // ── entering the zone ──
            {
                inZone  = true;
                minDist = d;
                minIdx  = i;
            }
            else if (d < minDist) // ── still in zone, update minimum ──
            {
                minDist = d;
                minIdx  = i;
            }
        }
        else if (inZone)          // ── leaving the zone ──
        {
            commitCrossing();
            inZone  = false;
            minDist = std::numeric_limits<double>::max();
        }
    }

    if (inZone)                   // ── track ended while still in zone ──
        commitCrossing();

    return crossings;
}

} // namespace detail

// ─────────────────────────────────────────────────────────────────────────────
//  Public API
// ─────────────────────────────────────────────────────────────────────────────

/// Detects all complete laps in a GPS track and returns structured lap data.
///
/// @param lat, lon, ts     Parallel vectors of the same length representing the
///                         GPS track.  `ts` values must be strictly increasing.
/// @param finishLat/Lon    Coordinates of the finish / start-finish line.
/// @param intermLat/Lon    Coordinates of intermediate checkpoints, in the
///                         expected order of passage within a lap.
/// @param thresholdM       Radius (metres) around each checkpoint within which
///                         a sample is considered a crossing.  Tune this to be
///                         slightly larger than the maximum expected GPS shift.
///                         Typical value: 15–25 m.
/// @param minLapTimeSec    Minimum plausible lap duration (seconds).  Finish-line
///                         crossings that are closer together than this are
///                         treated as GPS wobble and merged / discarded.
///
/// @return  One `single_lap` per complete lap (requires ≥ 2 finish crossings).
///          `intermediate_time` entries are elapsed seconds from lap start;
///          a value of -1.0 means that checkpoint was not detected in that lap.
///          `list_ts` contains absolute timestamps in chronological order:
///          [ lapStart, <detected intermediates in order>, lapEnd ]
std::vector<single_lap> findLaps(
    const std::vector<double>& lat,
    const std::vector<double>& lon,
    const std::vector<double>& ts,
    double finishLat,  double finishLon,
    const std::vector<double>& intermLat,
    const std::vector<double>& intermLon,
    double thresholdM,
    double minLapTimeSec)
{
    // ── Input validation ─────────────────────────────────────────────────────
    if (lat.size() != lon.size() || lat.size() != ts.size())
        throw std::invalid_argument("lat, lon and ts must have the same size");
    if (intermLat.size() != intermLon.size())
        throw std::invalid_argument("intermLat and intermLon must have the same size");
    if (lat.empty())
        return {};

    // ── 1. Detect finish-line crossings ──────────────────────────────────────
    // Pass minLapTimeSec as the debounce gap: two finish crossings within
    // minLapTimeSec of each other are almost certainly GPS noise.
    std::vector<double> finishTs = detail::findCrossings(
        lat, lon, ts,
        finishLat, finishLon,
        thresholdM,
        minLapTimeSec);

    if (finishTs.size() < 2)
        return {}; // not enough crossings for even one complete lap

    // ── 2. Detect intermediate crossings ─────────────────────────────────────
    const std::size_t nInterm = intermLat.size();
    std::vector<std::vector<double>> intermTs(nInterm);
    for (std::size_t k = 0; k < nInterm; ++k)
        intermTs[k] = detail::findCrossings(
            lat, lon, ts,
            intermLat[k], intermLon[k],
            thresholdM,
            0.0 /*no extra debounce for intermediates*/);

    // ── 3. Assemble laps ─────────────────────────────────────────────────────
    std::vector<single_lap> laps;
    laps.reserve(finishTs.size() - 1);

    for (std::size_t i = 0; i + 1 < finishTs.size(); ++i)
    {
        const double lapStart = finishTs[i];
        const double lapEnd   = finishTs[i + 1];

        single_lap lap;
        lap.lap_time = lapEnd - lapStart;
        lap.list_ts  = { lapStart, lapEnd };       // will insert intermediates below
        lap.intermediate_time.reserve(nInterm);

        // For each checkpoint, find the FIRST crossing inside [lapStart, lapEnd].
        // intermTs[k] is sorted chronologically, so a linear scan with early-
        // exit is sufficient.
        for (std::size_t k = 0; k < nInterm; ++k)
        {
            double found = -1.0;
            for (double t : intermTs[k])
            {
                if (t <= lapStart) continue;  // before this lap
                if (t >= lapEnd)   break;     // after this lap (list is sorted)
                found = t;
                break;
            }

            if (found >= 0.0)
            {
                //lap.intermediate_time.push_back(found - lapStart);
                lap.intermediate_time.push_back(found - lap.list_ts[lap.list_ts.size()-2]);

                // Insert the absolute timestamp into list_ts in sorted order
                // (just before the final lapEnd entry).
                auto pos = std::lower_bound(lap.list_ts.begin(),
                                            lap.list_ts.end(), found);
                lap.list_ts.insert(pos, found);
            }
            else
            {
                lap.intermediate_time.push_back(-1.0); // not detected this lap
            }
        }
        //If we have at least one intermediate time, add the last one
        if (lap.list_ts.size() > 2)
            lap.intermediate_time.push_back(lap.list_ts[lap.list_ts.size()-1] - lap.list_ts[lap.list_ts.size()-2]);

        lap.s_lap_time = time_to_string(lap.lap_time);
        for (size_t j = 0; j < lap.intermediate_time.size(); j++)
            lap.s_intermediate_time.emplace_back(time_to_string(lap.intermediate_time[j]));

        laps.push_back(std::move(lap));
    }
    return laps;
}