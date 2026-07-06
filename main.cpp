//TODO When the image with the track is rotated, there is probably an issue. I need to test that.
//TODO The cli for the number of frame when processing is not working properly
//TODO I should probably move the lap time display a bit.
//TODO Create an overlay for the speed
//TODO This stuff is slow as hell.
//TODO Limit the number of lap time displayed to 5, and keep the best displayed
//TODO Add the name of the track somewhere.
//TODO Add the locations of the intermediates on the track.
//TODO Modify merge audio video to take string instead of const char*
//TODO Convert the track data to a json file.
//TODO Display the speed on the bottom left using the cv::elipse.

#include <iostream>
#include <cstdio>
#include <vector>
#include <fstream>
#include <chrono>
#include <sstream>
#include <deque>
#include <filesystem>

#include <opencv2/opencv.hpp>

#include "extract_GPMF.h"
//#include "overlay_functions.h"
#include "compute_laps.h"
#include "merge_audio.h"
#include "merge_audio_video.h"
#include "lean_functions.hpp"
#include "overlay_functions_cairo.hpp"

using namespace std;
namespace fs = std::filesystem;
using namespace cv;
using namespace std::chrono;



std::vector<double> MovingAverage(const std::vector<double>& data, std::size_t windowSize) {
    std::vector<double> result;
    if (data.size() < windowSize || windowSize == 0) {
        return result; // Return empty result if window size is invalid
    }

    std::deque<double> window;
    double sum = 0.0;

    //To keep the same size, we just copy the average of the first few/last points.
    for (size_t i = 0; i < windowSize/2; i++) sum += data[i];
    sum /= windowSize/2;
    for (size_t i = 0; i < windowSize/2; i++) result.push_back(sum);
    sum = 0.0;

    for (std::size_t i = 0; i < data.size(); ++i) {
        window.push_back(data[i]);
        sum += data[i];

        if (window.size() > windowSize) {
            sum -= window.front();
            window.pop_front();
        }

        if (window.size() == windowSize) {
            result.push_back(sum / windowSize);
        }
    }

    //To keep the same size, we just copy the first few/last points.
    sum = 0.0;
    for (size_t i = 0; i < windowSize/2; i++) sum += data[i];
    sum /= windowSize/2;
    for (size_t i = data.size()-windowSize/2; i < data.size(); i++) result.push_back(sum);

    return result;
}


void convert_ms2kmh(extracted_data &data) {
    /* If the measurement are in m/s for the gps speed, change them to km/h
     *
     */
    //cerr << "unit: " << data.gps_speed_unit << endl;
    //cerr << "unit: " << data.gps_speed2_unit << endl;
    if (data.gps_speed_unit == "m/s")
        for (size_t i = 0; i < data.gps_speed.size(); i++) {
            data.gps_speed[i] = static_cast<int>(36*data.gps_speed[i])/10.0;
        }
    if (data.gps_speed2_unit == "m/s")
        for (size_t i = 0; i < data.gps_speed2.size(); i++) {
            data.gps_speed2[i] = static_cast<int>(36*data.gps_speed2[i])/10.0;
        }
}


void generate_output_name(global_struct &gs){
    //Generate an output name based on the track name and gps time.

    int cpt = 0;
    fs::path path = gs.paths_in[0]; 
    fs::path dir = path.parent_path();
    string name, tn;
    if (gs.laps.track_name.empty())
        tn = "Unknown_track";
    else
        tn = gs.laps.track_name;
    name = tn + "_" + gs.data.start_ts + ".MP4";        
    gs.path_output = dir.string() + "/" + name;        
    while (fs::exists(gs.path_output)){
        name = tn + "_" + gs.data.start_ts + "_" + to_string(cpt++) + ".MP4";
        gs.path_output = dir.string() + "/" + name;
    }
    cerr << "Output Name: " << name << endl;

}

int process_several_videos(global_struct &gs){
    //Since the gopro tends to cut the videos into smaller one, we have several input, a single output.
    //We assume that they are given in the proper order and that they have the same frame information.

    ///TODO finish the audio auto cut (mostly the end)
    ///TODO Replace the call of this function by a struct with all the data.
    ///TODO add other struct into the main struct (speed overlay, gps_data...)

    auto start = high_resolution_clock::now();
    VideoCapture cap(gs.paths_in[0]);
    if (!cap.isOpened()) {
        cout << "Error opening video stream or file: " << gs.paths_in[0] << endl;
        return -1;
    }
    cout << "Video " << gs.paths_in[0] << " is opened" << endl;

    int cpt = 0;
    int frame_width = static_cast<int>(cap.get(CAP_PROP_FRAME_WIDTH));
    int frame_height = static_cast<int>(cap.get(CAP_PROP_FRAME_HEIGHT));
    int ex = static_cast<int>(cap.get(CAP_PROP_FOURCC));
    double fps = cap.get(CAP_PROP_FPS);    
    bool overlay_lap_time = gs.path_tracks.empty() ? false : true;
    bool overlay_speed = true; //Todo Add an option to add or not the speed
    //Close the video in, we now know the frame size for the output.
    cap.release();

    //Used if we want to auto crop around the computed laps
    gs.crop_start_ts = -1;
    gs.crop_end_ts = -1;
    
    gs.data = extracted_data{};
    gs.gps = gps_data{};
    gs.laps = laps_data{};
    
    //Extract the data from the videos
    for (auto path: gs.paths_in)
        get_mp4_data(path.c_str(), gs.data);

    cout << "Starting video time: " << gs.data.start_ts << endl;

    //write_mp4_all_metadata(gs.paths_in[0]+".csv", gs.data, 0);
    //exit(0);

    //Extract the information of the track for the lap-time and intermediates times.
    if (overlay_lap_time) {        
        extract_lap_info(gs.path_tracks, gs.data, gs.laps);
        gs.gps.track_name = gs.laps.track_name;        
        // If we want to auto crop, we need to find when to start and to end.
        if (gs.auto_crop && !gs.laps.list_laps.empty()){
            gs.crop_start_ts = max(0.0, gs.laps.list_laps[0].list_ts[0]-gs.crop_shift_ts);            
            gs.crop_end_ts = gs.laps.list_laps.back().list_ts.back() + gs.crop_shift_ts;
            cerr << "All laps, Auto cropping from : " << gs.crop_start_ts << " to " << gs.crop_end_ts << endl;
        }
        if (gs.best_lap && !gs.laps.list_laps.empty()){
            //Find the best lap, the set the crop value
            double min_time=DBL_MAX;
            size_t min_index = 0;
            for (size_t i = 0; i < gs.laps.list_laps.size(); i++)
                if (gs.laps.list_laps[i].lap_time < min_time){
                    min_time = gs.laps.list_laps[i].lap_time;
                    min_index = i;
                }
            gs.crop_start_ts = max(0.0, gs.laps.list_laps[min_index].list_ts[0]-gs.crop_shift_ts);            
            gs.crop_end_ts = gs.laps.list_laps[min_index].list_ts.back() + gs.crop_shift_ts;
            cerr << "Best laps, Auto cropping from : " << gs.crop_start_ts << " to " << gs.crop_end_ts << endl;

        }
    }

    if (gs.path_output.empty())//Generate the name from the track name and date.        
        generate_output_name(gs);

    if (gs.time_only) return 0; //We computed the lap time, we do not want to process the video
 
    //Test the lean angle based only on the gps data
    //auto lean_result = lean_estimator(data.gps_lat, data.gps_long, data.gps_alt, data.gps_ts, 5.0);
    auto lean_result = lean_estimator(gs.data, 5.0);    

    convert_ms2kmh(gs.data);

    //Create the cairo HUD
    cv::Mat cairo_hud = create_static_hud(frame_width, frame_height);    
    if (overlay_speed) create_speed_hud(cairo_hud);    
    if (overlay_speed) create_lean_hud(cairo_hud);
    if (gs.overlay_track) create_track_hud(cairo_hud, gs.data, gs.gps, gs.laps);

    //Open the output video
    VideoWriter outputVideo;  // Open the output
    outputVideo.open(gs.temp_video, ex, fps, cv::Size(frame_width, frame_height), true);

    if (!outputVideo.isOpened()) {
        cerr << "Could not open the output video file for write: " << gs.temp_video << endl;
        return -1;
    }
    
    for (auto path: gs.paths_in){
        cap.open(path);
        if(!cap.isOpened()){
            cout << "Error opening video stream or file: " << path << endl;
            return -1;
        }
        cout << "Video " << path << " is opened" << endl;

        while(1){            
            if (cpt%200 == 199) {
                auto inter = high_resolution_clock::now();
                auto dur = duration_cast<milliseconds>(inter - start);
                cout << "Frame [" << cpt+1 << "/" << min(gs.data.nb_frames, gs.limit_frame) << "] (" << 100*(cpt+1)/min(gs.data.nb_frames, gs.limit_frame) << "%) ";
                cout << "fps: " << 1000.0*cpt/dur.count() << "\t\r" << std::flush;
            }
            cv::Mat frame, overlay = cairo_hud.clone();
            cap >> frame;
            if (frame.empty())
                break;
            //Display the speed on the frame
            double curr_ts = static_cast<double>(cpt++)/gs.data.framerate;
            //If we have auto crop, check if we are before the first lap, or after the last lap
            if ((gs.auto_crop || gs.best_lap) && !gs.laps.list_laps.empty()){
                if (curr_ts < gs.crop_start_ts || curr_ts > gs.crop_end_ts)
                    continue;                
            }
                
            if (overlay_lap_time) {                
                overlay = draw_laptime_hud(overlay, gs.laps, curr_ts);
            }
            //Overlay the track.
            if (gs.overlay_track) {                
                //overlay = draw_track_hud(overlay, gs.gps, curr_ts);
                //overlay = draw_track_hud_intermediate(overlay, gs.gps, gs.laps, curr_ts);
                overlay = draw_track_hud_lean(overlay, gs.gps, gs.laps, lean_result, curr_ts);
            }
            //Overlay the speed meter
            if (overlay_speed) {                
                size_t idx = 0;
                while(lean_result.ts[idx] < curr_ts && idx < lean_result.ts.size()-1) idx++;
                double l = -lean_result.lean_angle[idx];
                //idx = 0;
                //while(data.gps_ts[idx] < curr_ts && idx < data.gps_ts.size()-1) idx++;
                double s = gs.data.gps_speed[idx];

                overlay = draw_speed_hud(overlay, s);                
                overlay = draw_lean_hud(overlay, l);                

            }
            //Overlay the HUD
            overlay_hud(frame, overlay);

            outputVideo << frame; 
            if (cpt >= gs.limit_frame) {
                cerr << "Reached the requested number of frames. Stopping here" << endl;
                break;
            }
        }
        cout << "Releasing the capture" << endl;
        cap.release();
    }

    outputVideo.release();
    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<seconds>(stop - start);
    cout << "Video processing time: " << duration.count() << " seconds" << endl;   

    //Second extract the audio from the input videos.
    if (concatenate_audio_streams(gs.paths_in, gs.temp_audio, gs.crop_start_ts, gs.crop_end_ts) == 0)
    //Finally concatenante both into the output file
    concatenate_audio_video(gs.temp_video.c_str(), gs.temp_audio.c_str(), gs.path_output.c_str());
    cout << "Done concatenating" << endl;
    cout << "Removign: " << gs.temp_video << ", " << gs.temp_audio << endl;
    //Remove the temp files
    std::remove(gs.temp_video.c_str());
    std::remove(gs.temp_audio.c_str());

    cout << "Done removing temps" << endl;


    return 0;

}



void test(){


    string path_tracks = "./tracks/tracks.csv";
    string path_in = "./videos/20240718_anneau_1.MP4";
    string path_out = "/home/yohan/Documents/gopro/done/20240718_anneau_10_mod.MP4";
    string path_merged = "/home/yohan/Documents/gopro/done/20240718_anneau_10_audio.MP4";

    string path_out_audio = "/home/yohan/Documents/gopro/done/audio.aac";

    vector<string> list_path_in = {
            "/home/yohan/Documents/gopro/done/20240718_anneau_10.MP4",
           // "/home/yohan/Documents/gopro/done/20240718_anneau_9_2.MP4"
    };

    //demo_function(argc, argv);
    //demo_yohan(argc, argv);
    //read_video();
    //extracted_data data;
    //get_mp4_data("/home/yohan/Documents/gopro/done/20240715Mirecourt_2.MP4", data);
    //write_mp4_metadata("./output/metadata_mirecourt.txt", data);
    //read_write_video();
    //test_compute_lap();
    //test_show_lap_position();
    //test_min_distance();

    //concatenate_audio_streams(list_path_in, path_out_audio);
    //concatenate_audio_video(path_out.c_str(), path_out_audio.c_str(), path_merged.c_str());
    //concatenate_audio_streams(list_path_in[0].c_str(), list_path_in[1].c_str(), path_out_audio.c_str());
    //data_to_map_to_intermediate();
    //test_write_gps_data();        

}

void show_help(){

    cout << "To use the software:\n";
    cout << "   -i: path to input video. Can be used several times.\n";
    cout << "       Video will be processed in given order then merged\n";
    cout << "   -o: Path to output video.\n";
    cout << "   -t: path to file containing tracks information\n";
    cout << "   -no_overlay: Only show the lap times.\n";
}


std::vector<std::string> getGoProParts(const std::string& fullPath) {
    std::vector<std::string> parts;

    fs::path inputPath(fullPath);

    // Extract directory and filename
    fs::path dir = inputPath.parent_path();
    std::string filename = inputPath.filename().string();

    // Basic validation
    if (filename.size() != 12 || filename.substr(0, 2) != "GH") {
        return parts;
    }    
    // Extract parts:
    // GH010001.MP4
    //   ^^ -> part number
    //     ^^^^ -> clip number
    std::string partStr = filename.substr(2, 2);
    std::string clipStr = filename.substr(4, 4);
    std::string extension = filename.substr(8); // ".MP4"

    int startPart = std::stoi(partStr);

    // Loop through possible parts (01 → 99)
    for (int part = startPart; part <= 99; ++part) {
        std::ostringstream oss;
        oss << "GH"
            << std::setw(2) << std::setfill('0') << part
            << clipStr
            << extension;

        fs::path candidate = dir / oss.str();     
        if (fs::exists(candidate)) {
            parts.push_back(candidate.string());
        } else {
            // Stop when a part is missing (sequence ends)
            break;
        }
    }

    return parts;
}


void parse_input_folder(const string &path_in, vector<vector<string> > &paths_in, vector<string> &paths_out){

    //Take a folder, get all the first parts of the Gopro videoL GH01XXXX.MP4, then get all the following parts.
    //Generate the output names.
    //
    vector<string> first_files;

    cout << "Parsing " << path_in << " folder" << endl;
    
    for (const auto & entry : fs::directory_iterator(path_in)){        
        //Get the full path, then the filename
        fs::path entry_path(entry);
        string p = entry.path();
        string f = entry_path.filename().string();
        string ext = entry_path.extension();
        
        if (f.substr(0,4) == "GH01" && (ext == ".MP4" || ext == ".mp4")){
            //We keep this one, generate the output name at the same time
            first_files.push_back(p);
            //string path_out = path_in + f.substr(0, f.size()-ext.size()) + "processed" + ext;
            //paths_out.push_back(path_out);
        }
    }

    sort(first_files.begin(), first_files.end());
    sort(paths_out.begin(), paths_out.end());

    for (auto f: first_files){        
        vector<string> curr = getGoProParts(f);
        if (curr.empty()) continue;
        paths_in.push_back(curr);
        cout << f << "->" << curr.size() << endl;
    }


}

void process_full_folder(vector<vector<string> > &paths_in, vector<string> &paths_out, global_struct &gs){

    assert(paths_in.size() == paths_out.size());

    string all_times, best_time;
    double best = DBL_MAX;
    
    for (size_t i = 0; i < paths_in.size(); i++){                
        gs.paths_in = paths_in[i];        
        if (paths_out.size() > i)
            gs.path_output = paths_out[i];
        else
            gs.path_output = "";        
        cout << "Processing videos: \n";
        for (auto input: gs.paths_in)
            cout << "\t" << input << "\n";
        //cout << "Output: " << gs.path_output << endl;                
        process_several_videos(gs);
        //Show all the lap time, display the best one at the end.
        all_times += gs.path_output + "\n";        
        for (auto lap: gs.laps.list_laps){            
            string curr_time = "\t||" + lap.s_lap_time + "|";            
            for (auto inter: lap.s_intermediate_time)
                curr_time += "|" + inter;            
            curr_time += "|\n";
            all_times += curr_time;
            if (lap.lap_time < best){best = lap.lap_time; best_time=curr_time;}            
        }

    }

    cout << "List of all lap time:\n" << all_times << endl;
    cout << "\n-----BEST LAP TIME-----\n" << best_time << endl;

}


void parse_arguments(int argc, char* argv[]){

    global_struct gs;
    vector<string> arg;
    string path_all_videos;
    vector<vector<string> > paths_in;
    vector<string> paths_out;
        
    for (int i = 1; i < argc; i++){
        arg.emplace_back(argv[i]);
    }

    for (size_t i = 0; i < arg.size(); i++){
        if (arg[i] == "-i" && i < arg.size()-1){
            gs.paths_in.emplace_back(arg[i+1]);
        }
        if (arg[i] == "-o" && i < arg.size()-1){            
            gs.path_output = arg[i+1];
        }
        if (arg[i] == "-t" && i < arg.size()-1){            
            gs.path_tracks = arg[i+1];
        }
        if ((arg[i] == "-p" || arg[i] == "-path") && i < arg.size()-1){
            path_all_videos = arg[i+1];
        }
        if (arg[i] == "-no_overlay") {            
            gs.overlay_track=false;
        }
        if (arg[i] == "-auto_crop" || arg[i] == "-ac") {
            // Take five seconds before the first lap, and 5 seconds after the last            
            gs.auto_crop = true;
        }
        if (arg[i] == "-best_lap" || arg[i] == "-bl") {
            // Take five seconds before the best lap, and 5 seconds after it
            gs.best_lap = true;
        }
        if (arg[i] == "-time_only" || arg[i] == "-to") {
            // Take five seconds before the best lap, and 5 seconds after it
            gs.time_only = true;
        }
        if ((arg[i] == "-l" || arg[i] == "-limit") && i < arg.size()-1){
            //Limit the number of frame we extract from the video, useful for testing
            gs.limit_frame = stoi(arg[i+1]);            
        }
        if (arg[i] == "-nb_thread" && i < arg.size()-1){
            int nb_thread = stoi(arg[i+1]);
            cout << "Limiting the number of threads: " << nb_thread << endl;
            if (nb_thread >= 0)
                cv::setNumThreads(nb_thread);
        }            

    }

    if (gs.auto_crop && gs.best_lap){
        cout << "Warning, best lap and auto crop selected. Keeping auto crop" << endl;
        gs.best_lap = false;
    }

    if (!path_all_videos.empty()){        
        parse_input_folder(path_all_videos, paths_in, paths_out);
        if (paths_in.empty()){
            cout << "No video found in the given folder: " << path_all_videos << endl;
            return;
        }        
        process_full_folder(paths_in, paths_out, gs);
        return;
    }
    else{
        if(gs.paths_in.empty()){
            cerr << "No input given." << endl;
            show_help();
            return;
        }
        if(gs.path_output.empty()){
            cerr << "Warning, no output given." << endl;
            //show_help();
            //return;
        }

        if (gs.paths_in.size() == 1){
            cout << "Detect next video parts" << endl;
            gs.paths_in = getGoProParts(gs.paths_in[0]);
        }
    }

    cout << "Processing videos: \n";
    for (auto input: gs.paths_in)
        cout << "\t" << input << "\n";
    cout << "Output: " << gs.path_output << endl;
    //First create the overlay of the video in a temp file
    int success = process_several_videos(gs);    
    
}


int main(int argc, char* argv[]) {

    char read_attemps[]="OPENCV_FFMPEG_READ_ATTEMPTS=800000";
    putenv(read_attemps);
    //test(); return 0;
    parse_arguments(argc, argv);
    return 0;
}


