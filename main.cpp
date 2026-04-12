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
#include <vector>
#include <fstream>
#include <chrono>
#include <sstream>
#include <deque>
#include <filesystem>

#include <opencv2/opencv.hpp>

#include "extract_GPMF.h"
#include "overlay_functions.h"
#include "compute_laps.h"
#include "merge_audio.h"
#include "merge_audio_video.h"

using namespace std;
namespace fs = std::filesystem;
using namespace cv;
using namespace std::chrono;

#ifndef M_PI
#define M_PI 3.14159265

#endif // !M_PI
#define d2r (M_PI / 180.0)


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
#if 0
void compute_pitch_roll(extracted_data &data, lean_data &lean){

    /* This function is supposed to estimate the lean angle using the gyro and the accelerometer.
     * This is derived from my stuff done for the M5 stuff.
     * But here I already have all the data and I cannot do the calibration.
     *
     */

    double ratio_gyro = 0.95F;
    double ratio_accel = 1.0-ratio_gyro;
    double theta=0.0, thetaM, phi=0.0, phiM;
    double dt = 1.0/data.acclrate; //time dleta between two points
    size_t windows_size = 25;

    vector<double> fax, fay, faz, fgx, fgy, fgz; //Filtered version of accel and gyro.
    vector<double> roll, pitch;
    //We have about 200hz for these sensor, we we use 7 frames to filter.
    fax = MovingAverage(data.accl_x, windows_size);
    fay = MovingAverage(data.accl_y, windows_size);
    faz = MovingAverage(data.accl_z, windows_size);
    fgx = MovingAverage(data.gyro_x, windows_size);
    fgy = MovingAverage(data.gyro_y, windows_size);
    fgz = MovingAverage(data.gyro_z, windows_size);

    /*for (int i = 1; i < 10; i++) {
        cout << 100*i << ": ACCL [" << data.accl_x[100*i] << "," << data.accl_y[100*i] << "," << data.accl_z[100*i] << "]\n";
        cout << 100*i << ": ACCL [" << fax[100*i] << "," << fay[100*i] << "," << faz[100*i] << "]\n";
        cout << 100*i << ": GYRO [" << data.gyro_x[100*i] << "," << data.gyro_y[100*i] << "," << data.gyro_z[100*i] << "]\n";
        cout << 100*i << ": GYRO [" << fgx[100*i] << "," << fgy[100*i] << "," << fgz[100*i] << "]\n";
    }*/
    //This does not work, this makes no sense.
    for (size_t i = 0; i < fax.size(); i++) {
        //double curr_ax = fax[i];
        thetaM = -atan2(fax[i],faz[i])/M_PI*180.0;
        phiM=-atan2(fay[i],faz[i])/M_PI*180;

        theta=(theta+fgy[i]*dt)*ratio_gyro + thetaM*ratio_accel;
        phi=(phi-fgx[i]*dt)*ratio_gyro + phiM*ratio_accel;
        //lean.pitch.emplace_back(phi);
        //lean.roll.emplace_back(theta);
        pitch.emplace_back(phi);
        roll.emplace_back(theta);
    }

    //Now compute one pitch and one roll for each frames.
    pitch = MovingAverage(pitch, 5);
    roll = MovingAverage(roll, 5);

    for (size_t i = 0; i < data.nb_frames; i++) {
        size_t curr_pos = static_cast<size_t>((i+0.5)*data.nb_frames/pitch.size());
        lean.pitch.emplace_back(pitch[curr_pos]);
        lean.roll.emplace_back(roll[curr_pos]);
    }



    /*
        GetGyroData(250, gyro);
        GetAccelData(250, accel);
        float curr_accel[3], curr_gyro[3];
        for (int i = 0; i < 3; i++){
            curr_accel[i] = accel[i] - offset_accel[i];
            curr_gyro[i] = gyro[i] - offset_gyro[i];
        }
        thetaM=-atan2(curr_accel[0],curr_accel[2])/3.141592654*180;
        phiM=-atan2(curr_accel[1],curr_accel[2])/3.141592654*180;

        dt=(millis()-millisOld)/1000.0;
        millisOld=millis();
        //Theta = around Y axis
        theta=(theta+curr_gyro[1]*dt)*ratio_gyro+thetaM*ratio_accel;
        //Phi = around X axis
        phi=(phi-curr_gyro[0]*dt)*ratio_gyro+ phiM*ratio_accel;

        *pitch = phi;
        *roll = theta;*/

    if (true) {
        cv::Mat graph1 = cv::Mat::zeros(500, 2500, CV_8UC3);
        cv::Mat graph2 = cv::Mat::zeros(500, 2500, CV_8UC3);

        for (size_t i = 1; i < data.accl_x.size(); i++) {
            cv::line(graph1, cv::Point(i-1, 50+data.accl_x[i-1]),cv::Point(i, 50+data.accl_x[i]), cv::Scalar(255,0,0),1);
            cv::line(graph2, cv::Point(i-1, 50+fax[i-1]),cv::Point(i, 50+fax[i]), cv::Scalar(0,0,255),1);

            cv::line(graph1, cv::Point(i-1, 150+data.accl_y[i-1]),cv::Point(i, 150+data.accl_y[i]), cv::Scalar(255,0,0),1);
            cv::line(graph2, cv::Point(i-1, 150+fay[i-1]),cv::Point(i, 150+fay[i]), cv::Scalar(0,0,255),1);

            cv::line(graph1, cv::Point(i-1, 250+data.accl_z[i-1]),cv::Point(i, 250+data.accl_z[i]), cv::Scalar(255,0,0),1);
            cv::line(graph2, cv::Point(i-1, 250+faz[i-1]),cv::Point(i, 250+faz[i]), cv::Scalar(0,0,255),1);
        }

        cv::addWeighted(graph1, 0.5, graph2, 0.5, 0.0, graph1);
        cv::imwrite("./graph.png", graph1);
    }


}
#elif 1

void compute_pitch_roll(extracted_data &data, lean_data &lean) {

    double ALPHA = 0.98;
    double DT = 1.0/data.acclrate;
    double theta = 0.0;
    double theta_accel, theta_gyro;
    size_t windows_size = 25;
    double MPI180 = 180/M_PI;

    vector<double> fax, fay, faz, fgx, fgy, fgz; //Filtered version of accel and gyro.
    vector<double> roll, pitch;
    //We have about 200hz for these sensor, we we use 25 frames to filter.
    //We should probably replace by a better filter.
    fax = MovingAverage(data.accl_x, windows_size);
    fay = MovingAverage(data.accl_y, windows_size);
    faz = MovingAverage(data.accl_z, windows_size);
    fgx = MovingAverage(data.gyro_x, windows_size);
    fgy = MovingAverage(data.gyro_y, windows_size);
    fgz = MovingAverage(data.gyro_z, windows_size);

    for (size_t i = 0; i < fax.size(); i++) {
        /*theta_accel = atan2(fay[i], sqrt(fax[i] * fax[i] + faz[i] * faz[i]));
        theta_gyro = theta + fgx[i] * DT;
        theta = ALPHA * theta_gyro + (1.0 - ALPHA) * theta_accel;
        roll.push_back(theta*(180/M_PI));*/
        double r = atan2(fay[i] , faz[i]) * MPI180;
        double p = atan2((- fax[i]) , sqrt(fay[i] * fay[i] + faz[i] * faz[i])) * MPI180;
        roll.emplace_back(r);
        pitch.emplace_back(p);
    }

    for (size_t i = 0; i < data.nb_frames; i++) {
        size_t curr_pos = static_cast<size_t>((i+0.5)*data.nb_frames/roll.size());
        lean.pitch.emplace_back(pitch[curr_pos]);
        lean.roll.emplace_back(roll[curr_pos]);
    }


    if (true) {
        cv::Mat graph1 = cv::Mat::zeros(500, 2500, CV_8UC3);
        cv::Mat graph2 = cv::Mat::zeros(500, 2500, CV_8UC3);

        cv::line(graph1, cv::Point(0, 50),cv::Point(2500, 50), cv::Scalar(255,255,255),1);
        cv::line(graph1, cv::Point(0, 150),cv::Point(2500, 150), cv::Scalar(255,255,255),1);
        cv::line(graph1, cv::Point(0, 250),cv::Point(2500, 250), cv::Scalar(255,255,255),1);

        for (size_t i = 1; i < data.accl_x.size(); i++) {
            cv::line(graph1, cv::Point(i-1, 50+3*data.accl_x[i-1]),cv::Point(i, 50+3*data.accl_x[i]), cv::Scalar(255,0,0),1);
            cv::line(graph2, cv::Point(i-1, 50+3*fax[i-1]),cv::Point(i, 50+3*fax[i]), cv::Scalar(0,0,255),1);

            cv::line(graph1, cv::Point(i-1, 150+3*data.accl_y[i-1]),cv::Point(i, 150+3*data.accl_y[i]), cv::Scalar(255,0,0),1);
            cv::line(graph2, cv::Point(i-1, 150+3*fay[i-1]),cv::Point(i, 150+3*fay[i]), cv::Scalar(0,0,255),1);

            cv::line(graph1, cv::Point(i-1, 250+3*data.accl_z[i-1]),cv::Point(i, 250+3*data.accl_z[i]), cv::Scalar(255,0,0),1);
            cv::line(graph2, cv::Point(i-1, 250+3*faz[i-1]),cv::Point(i, 250+3*faz[i]), cv::Scalar(0,0,255),1);
        }

        cv::addWeighted(graph1, 0.5, graph2, 0.5, 0.0, graph1);
        cv::imwrite("./graph.png", graph1);
    }

}
#endif

void read_video() {

    string path_video = "./videos/hero7.mp4";

    VideoCapture cap(path_video);

    if(!cap.isOpened()){
        cout << "Error opening video stream or file" << endl;
        return;
    }
    cout << "Video is opened" << endl;
    int cpt = 0;

    while(1){

        Mat frame;
        // Capture frame-by-frame
        cap >> frame;
        // If the frame is empty, break immediately
        if (frame.empty())
            break;
        //cerr << frame.size() << endl;
        cpt++;
        cv::imwrite("./images/image_"+to_string(cpt)+".png", frame);

        if (cpt > 20) break;
    }
    cout << "Number of frames: " << cpt << endl;
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

void get_list_index(double fps, double data_rate, int frame_id, double start_shift, int &start, int &end) {
    /* Compute the first and last data point corresponding to a given frame.
     * fps: The video framerate.
     * data_rate: The number of points per second for the current sensor (ex. 18hz for gps)
     * frame_id: The frame we want to compute for.
     * start_shift: Shift to apply if the sensor started before or after the first frame.
     * start: id of the first data point.
     * end: id of the last data point.
     */

    double point_per_frame = data_rate/fps;
    double start_frame = (1.0*frame_id + start_shift)*point_per_frame;
    end = static_cast<int>(start_frame + point_per_frame);
    start = static_cast<int>(start_frame);
    if (end == start) end++; //We need at least one frame.
}

double get_mean_value(const extracted_data &data, const string &type, const int &start, const int &end) {
    /* Take a structure of data points. The type of data we are looking for, the start and end points
     * Compute the average accross all the data and return it.
     */
    vector<double> points;
    if (end - start < 1) {
        cerr << "Error with input frames." << endl;
        return NAN;
    }
    if (type == "gps_lat")
        points = vector<double>(data.gps_lat.begin()+start, data.gps_lat.begin()+end);
    else if (type == "gps_long")
        points = vector<double>(data.gps_long.begin()+start, data.gps_long.begin()+end);
    else if (type == "gps_alt")
        points = vector<double>(data.gps_alt.begin()+start, data.gps_alt.begin()+end);
    else if (type == "gps_speed")
        points = vector<double>(data.gps_speed.begin()+start, data.gps_speed.begin()+end);
    else if (type == "gps_speed2")
        points = vector<double>(data.gps_speed2.begin()+start, data.gps_speed2.begin()+end);
    else if (type == "accl_x")
        points = vector<double>(data.accl_x.begin()+start, data.accl_x.begin()+end);
    else if (type == "accl_y")
        points = vector<double>(data.accl_y.begin()+start, data.accl_y.begin()+end);
    else if (type == "accl_y")
        points = vector<double>(data.accl_z.begin()+start, data.accl_z.begin()+end);
    else if (type == "gyro_x")
        points = vector<double>(data.gyro_x.begin()+start, data.gyro_x.begin()+end);
    else if (type == "gyro_y")
        points = vector<double>(data.gyro_y.begin()+start, data.gyro_y.begin()+end);
    else if (type == "gyro_z")
        points = vector<double>(data.gyro_z.begin()+start, data.gyro_z.begin()+end);
    else {
        cerr << "Error unknown type." << endl;
        return NAN;
    }

    double mean_val = 0.0;
    for (auto p: points) {
        cerr << "Curr value: " << p << endl;
        mean_val += p;
    }
    mean_val/=static_cast<double>(points.size());
    return mean_val;
}

void read_write_video(){

    auto start = high_resolution_clock::now();
    string video_in = "/home/yohan/Documents/C++/GoProMoto/videos/GH039080.MP4";
    //string video_in = "/home/yohan/Documents/C++/GoProMoto/videos/hero7.MP4";
    string video_out = "/home/yohan/Documents/C++/GoProMoto/videos/GH039080_out.MP4";

    //video_in = "/home/yohan/Documents/gopro/merged/20240718_anneau_1.MP4";
    //video_out = "/home/yohan/Documents/gopro/merged/20240718_anneau_1_mod.MP4";

    video_in = "./videos/20240716_anneau_1_2.MP4";
    video_out = "./videos/20240716_anneau_1_2_mod.MP4";

    VideoCapture cap(video_in);

    if(!cap.isOpened()){
        cout << "Error opening video stream or file" << endl;
        return;
    }
    cout << "Video is opened" << endl;

    int cpt = 0;
    int frame_width = static_cast<int>(cap.get(CAP_PROP_FRAME_WIDTH));
    int frame_height = static_cast<int>(cap.get(CAP_PROP_FRAME_HEIGHT));

    extracted_data data;
    gps_data gps;
    get_mp4_data(video_in.c_str(), data);
    create_gps_object(gps, data, frame_width/3, frame_height/3);
    convert_ms2kmh(data);

    lean_data lean;
    compute_pitch_roll(data, lean);

    cerr << "gyro unit: " << data.gyro_x_unit << endl;
    //return;

    if (data.nb_frames < 1) {
        cerr << "No data extracted from the frames" << endl;
        return;
    }
    cerr << "Number of frames: " << data.nb_frames << endl;

    int ex = static_cast<int>(cap.get(CAP_PROP_FOURCC));
    char EXT[] = {(char)(ex & 0XFF) , (char)((ex & 0XFF00) >> 8),(char)((ex & 0XFF0000) >> 16),(char)((ex & 0XFF000000) >> 24), 0};
    Size S = Size((int) cap.get(CAP_PROP_FRAME_WIDTH), (int) cap.get(CAP_PROP_FRAME_HEIGHT));

    VideoWriter outputVideo;  // Open the output
    outputVideo.open(video_out, ex, cap.get(CAP_PROP_FPS), S, true);

    if (!outputVideo.isOpened()) {
        cerr << "Could not open the output video file for write\n";
        return;
    }

    while(1){
        Mat frame;
        cap >> frame;
        if (frame.empty())
            break;
        //Convert to rgba
        cv::cvtColor(frame, frame, cv::COLOR_BGR2BGRA);
        //Display the speed on the frame
        int start_index, end_index;

        get_list_index(data.framerate, data.gpsrate, cpt, data.gps_start, start_index, end_index);
        double speed = get_mean_value(data, "gps_speed", start_index, end_index);
        cerr << "Computed speed at frame " << cpt << ": " << speed << " Start index:" << start_index << ", end: " << end_index << endl;
        cv::putText(frame, to_string(speed)+"Km/h", cv::Point(50, 50), cv::FONT_HERSHEY_DUPLEX, 2.0, cv::Scalar(118, 185, 0, 255), 2);
        //cv::putText(frame, "Pitch "+ to_string(lean.pitch[cpt]), cv::Point(500, 50), cv::FONT_HERSHEY_DUPLEX, 2.0, cv::Scalar(118, 185, 0, 255), 2);
        //cv::putText(frame, "Roll "+ to_string(lean.roll[cpt]), cv::Point(1000, 50), cv::FONT_HERSHEY_DUPLEX, 2.0, cv::Scalar(118, 185, 0, 255), 2);

        cv::Mat sub_frame = frame(cv::Rect(100,100,gps.blank_map.cols, gps.blank_map.rows));
        display_position_gps(gps, data, cpt);
        cv::addWeighted(sub_frame, 0.4, gps.position_map, 0.6, 0.0, sub_frame);

        cpt++;
        //Back to the rgb space
        cv::cvtColor(frame, frame, cv::COLOR_BGRA2BGR);
        outputVideo << frame;
        if (cpt > 1000) break;
    }
    //Close the writer.
    outputVideo.release();

    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<milliseconds>(stop - start);
    cout << "Video processing time: " << duration.count() << endl;

}


void process_several_videos(vector<string> &paths_in, const string &path_out, bool overlay_track, string path_tracks_info) {
    //Since the gopro tends to cut the videos into smaller one, we have several input, a single output.
    //We assume that they are given in the proper order and that they have the same frame information.

    auto start = high_resolution_clock::now();
    VideoCapture cap(paths_in[0]);
    if (!cap.isOpened()) {
        cout << "Error opening video stream or file: " << paths_in[0] << endl;
        return;
    }
    cout << "Video " << paths_in[0] << " is opened" << endl;

    int cpt = 0;
    int frame_width = static_cast<int>(cap.get(CAP_PROP_FRAME_WIDTH));
    int frame_height = static_cast<int>(cap.get(CAP_PROP_FRAME_HEIGHT));
    int ex = static_cast<int>(cap.get(CAP_PROP_FOURCC));
    double fps = cap.get(CAP_PROP_FPS);
    bool overlay_track_time = path_tracks_info.empty() ? false : true;
    bool overlay_speed = true; //Todo Add an option to add or not the speed
    //Close the video in, we now know the frame size for the output.
    cap.release();

    speed_overlay speed;
    create_speed_meter(speed, frame_height / 3, frame_height / 3);
    //display_speed(speed, static_cast<double> (i));
    extracted_data data;
    gps_data gps;
    laps_data laps;
    //Extract the data from the videos
    for (auto path: paths_in)
        get_mp4_data(path.c_str(), data);

    //Extract the information of the track for the lap-time and intermediates times.
    if (overlay_track_time) {
        extract_lap_info(path_tracks_info, data, laps);
        gps.track_name = laps.track_name;
    }

    //Create the object with the track overlay
    if (!create_gps_object(gps, data, frame_width / 3, frame_height / 3)){
        overlay_track = false;
        overlay_speed = false;
    }
    convert_ms2kmh(data);

    //Open the output video
    VideoWriter outputVideo;  // Open the output
    outputVideo.open(path_out, ex, fps, cv::Size(frame_width, frame_height), true);

    if (!outputVideo.isOpened()) {
        cerr << "Could not open the output video file for write: " << path_out << endl;
        return;
    }

    for (auto path: paths_in){
        cap.open(path);
        if(!cap.isOpened()){
            cout << "Error opening video stream or file: " << path << endl;
            return;
        }
        cout << "Video " << path << " is opened" << endl;

        while(1){
            //if (cpt%200 == 199) cout << "Frame [" << cpt+1 << "/" << data.nb_frames << "] (" << 100*(cpt+1)/data.nb_frames << "%)" << endl;
            if (cpt%200 == 199) {
                auto inter = high_resolution_clock::now();
                auto dur = duration_cast<milliseconds>(inter - start);
                cout << "Frame [" << cpt+1 << "/" << data.nb_frames << "] (" << 100*(cpt+1)/data.nb_frames << "%) ";
                cout << "fps: " << 1000.0*cpt/dur.count() << "\t\r" << std::flush;
            }
            cv::Mat frame, sub_frame;
            cap >> frame;
            if (frame.empty())
                break;
            //Convert to rgba
            cv::cvtColor(frame, frame, cv::COLOR_BGR2BGRA);
            //Display the speed on the frame
            int start_index, end_index;
            get_list_index(data.framerate, data.gpsrate, cpt, data.gps_start, start_index, end_index);
            if (overlay_track_time) {
                cv::Mat lap_time_overlay = display_lap_time(laps, start_index);
                sub_frame = frame(cv::Rect(frame.cols - lap_time_overlay.cols-50, frame.rows - lap_time_overlay.rows-50,
                                           lap_time_overlay.cols, lap_time_overlay.rows));
                cv::addWeighted(sub_frame, 0.4, lap_time_overlay, 0.6, 0.0, sub_frame);
            }
            //Overlay the track.
            if (overlay_track) {
                sub_frame = frame(cv::Rect(100, 100, gps.blank_map.cols, gps.blank_map.rows));
                display_position_gps(gps, data, start_index);
//                cerr << sub_frame.size() << "  :  " << gps.position_map.size() << endl;
//                cerr << sub_frame.type() << "  :  " << gps.position_map.type() << endl;
//                cerr << sub_frame.channels() << "  :  " << gps.position_map.channels() << endl;
                cv::addWeighted(sub_frame, 0.4, gps.position_map, 0.6, 0.0, sub_frame);
            }
            //Overlay the speed meter
            if (overlay_speed) {
                display_speed(speed, data.gps_speed[start_index]);
                sub_frame = frame(cv::Rect(100, frame.rows-speed.curr_speed.rows-20,
                                           speed.curr_speed.cols, speed.curr_speed.rows));
                display_position_gps(gps, data, start_index);
                cv::addWeighted(sub_frame, .7, speed.curr_speed, 1.0, 0.0, sub_frame);
            }
            //cv::imshow("frame", frame);
            //cv::waitKey();

            cpt++;
            //Back to the rgb space
            cv::cvtColor(frame, frame, cv::COLOR_BGRA2BGR);
            outputVideo << frame;
            //if (cpt > 8000) break;
        }
        cout << "Releasing the capture" << endl;
        cap.release();
    }

    outputVideo.release();
    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<seconds>(stop - start);
    cout << "Video processing time: " << duration.count() << " seconds" << endl;


}

void data_to_map_to_intermediate(){
    //The goal of this function is to help to determine the important gps coordinate of a track using the computed map.
    // This is for track with improper gps images on gmaps.

    string video_in = "/home/yohan/Documents/gopro/done/20240715Mirecourt_2.MP4";
    extracted_data data;
    gps_data gps;
    get_mp4_data(video_in.c_str(), data);
    create_gps_object(gps, data, 1000, 1000);

    //loop over indices, display the position, when clicked, next image with 100+ index.
    for (int i = 0; i < (int)data.gps_lat.size(); i+=30){
        display_position_gps(gps, data, i);
        cout.precision(17);
        cout << "Position [" << data.gps_lat[i] << "," << data.gps_long[i] << "]" << endl;
        cv::imshow("gps position", gps.position_map);
        cv::waitKey();
    }

}

void test_compute_lap(){

    string video_in = "./videos/20240718_anneau_1.MP4";
    string video_in2 = "./videos/20240718_anneau_1_2.MP4";
    extracted_data data;
    gps_data gps;
    laps_data laps;
    get_mp4_data(video_in.c_str(), data);
    get_mp4_data(video_in2.c_str(), data);

    double start_lat = 47.95120010954625;
    double start_long = 7.416307636077316;

    extract_lap_info("./tracks/tracks.csv", data, laps);
    vector<size_t> lap_indexes = find_coordinates(data, start_lat, start_long, 10, 0, 0);

    for (auto i: lap_indexes){
        //cout << "Lap start: " << i << endl;
    }
    display_lap_time(laps, 3000);
    display_lap_time(laps, 5000);
    display_lap_time(laps, 7000);
    display_lap_time(laps, 9000);
    display_lap_time(laps, 10000);
    display_lap_time(laps, 12000);
}

void test_min_distance(){
    double lat, lon, min_dist = DBL_MAX;
    //lat = 48.32175; lon = 6.081345;
    //lat = 48.321204; lon = 6.0840587;
    lat = 48.320570; lon = 6.0825542;

    string path_video = "/home/yohan/Documents/gopro/done/20240715Mirecourt_2.MP4";

    extracted_data data;
    gps_data gps;
    laps_data laps;
    get_mp4_data(path_video.c_str(), data);

    for (size_t i = 0; i < data.gps_lat.size(); i++){
        double dist = haversine_meters(lat, lon, data.gps_lat[i], data.gps_long[i]);
        if (min_dist > dist) min_dist = dist;
    }

    cout << "Minimum distance: " << min_dist << "m" << endl;


}

void test_show_lap_position(){
    //I have issue with displaying the tracks and the position on it.
    //This function is to fix that.

    auto start = high_resolution_clock::now();
    string video_in = "/home/yohan/Documents/C++/GoProMoto/videos/GH039080.MP4";

    //video_in = "./videos/20240718_anneau_1.MP4";
    video_in = "./videos/20240716_anneau_1_2.MP4";

    extracted_data data;
    gps_data gps;
    get_mp4_data(video_in.c_str(), data);
    create_gps_object(gps, data, 1000, 1000);
    int cpt = 0;
    while(1){
        //if (cpt%10 == 9) cout << "Frame [" << cpt+1 << "/" << data.nb_frames << "] (" << 100*(cpt+1)/data.nb_frames << "%)" << endl;
        int start_index, end_index;
        get_list_index(data.framerate, data.gpsrate, cpt, data.gps_start, start_index, end_index);

        display_position_gps(gps, data, start_index);

        cv::imshow("track map", gps.position_map);
        cv::waitKey(10);
        cpt++;
    }
}

void test_write_gps_data(){

    // /home/yohan/Documents/gopro/GH019947.MP4
    // /home/yohan/Documents/gopro/GH029947.MP4
    string video_1 = "/home/yohan/Documents/gopro/GH019947.MP4";
    string video_2 = "/home/yohan/Documents/gopro/GH029947.MP4";

    extracted_data data;
    gps_data gps;
    gps.track_name = "Mirecourt";
    get_mp4_data(video_1.c_str(), data);
    get_mp4_data(video_2.c_str(), data);
    //Test, to correct the GPS data inconsistency. It seems that sometime the data jump around.

    create_gps_object(gps, data, 1000, 1000);
    write_mp4_metadata("./output/metadata_test.txt", data);

    cout << "Lat: [" << gps.min_lat << "," << gps.max_lat << "]\n";
    cout << "Long: [" << gps.min_long << "," << gps.max_long << "]\n";

    cout << "distance: " << haversine_meters(gps.min_lat, gps.min_long, gps.max_lat, gps.max_long) << endl;

    cv::imshow("Track", gps.blank_map);
    cv::waitKey();

}

void display_speed(){

    speed_overlay speed;
    create_speed_meter(speed, 500, 500);
    //display_speed(speed, static_cast<double> (i));

    cv::imshow("image", speed.blank_speed);
    cv::waitKey();

    //cv::ellipse(speed.blank_speed,speed.centre, cv::Size(speed.radius,speed.thickness), 0, 135, 225, mid, speed.thickness);
    for (size_t i = 0; i < 360; i+=20){
        size_t s = (i+270)%360;
        //speed.curr_speed = speed.blank_speed.clone();
        //cv::ellipse(speed.curr_speed,speed.centre, cv::Size(speed.radius,speed.thickness), s, 135, 225, cv::Scalar(0,225,125,255), speed.thickness);
        display_speed(speed, static_cast<double> (i));
        cerr << "Speed: " << i << endl;
        cv::imshow("image", speed.curr_speed);
        cv::waitKey();

    }

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
    //process_video(path_in, path_out, true, path_tracks);
    //process_several_videos(list_path_in, path_out, true, path_tracks);

    //concatenate_audio_streams(list_path_in, path_out_audio);
    //concatenate_audio_video(path_out.c_str(), path_out_audio.c_str(), path_merged.c_str());
    //concatenate_audio_streams(list_path_in[0].c_str(), list_path_in[1].c_str(), path_out_audio.c_str());
    //data_to_map_to_intermediate();
    //test_write_gps_data();
    display_speed();

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


void parse_arguments(int argc, char* argv[]){

    vector<string> arg, list_inputs;
    string path_output, path_tracks;
    string temp_video = "./temp_video.MP4", temp_audio = "./temp_audio.aac";
    bool overlay_track=true;
    for (int i = 1; i < argc; i++){
        arg.emplace_back(argv[i]);
    }

    for (size_t i = 0; i < arg.size(); i++){
        if (arg[i] == "-i" && i < arg.size()-1){
            list_inputs.emplace_back(arg[i+1]);
        }
        if (arg[i] == "-o" && i < arg.size()-1){
            path_output = arg[i+1];
        }
        if (arg[i] == "-t" && i < arg.size()-1){
            path_tracks = arg[i+1];
        }
        if (arg[i] == "-no_overlay") overlay_track=false;
    }

    if(list_inputs.empty()){
        cerr << "No input given." << endl;
        show_help();
        return;
    }
    if(path_output.empty()){
        cerr << "No output given." << endl;
        show_help();
        return;
    }

    if (list_inputs.size() == 1){
        cout << "Detect next video parts" << endl;
        list_inputs = getGoProParts(list_inputs[0]);
    }

    cout << "Processing videos: \n";
    for (auto input: list_inputs)
        cout << "\t" << input << "\n";
    cout << "Output: " << path_output << endl;
    //First create the overlay of the video in a temp file
    process_several_videos(list_inputs, temp_video, overlay_track, path_tracks);
    //Second extract the audio from the input videos.
    concatenate_audio_streams(list_inputs, temp_audio);
    //Finally concatenante both into the output file
    concatenate_audio_video(temp_video.c_str(), temp_audio.c_str(), path_output.c_str());

}


int main(int argc, char* argv[]) {

    char read_attemps[]="OPENCV_FFMPEG_READ_ATTEMPTS=800000";
    putenv(read_attemps);
    //test();
    parse_arguments(argc, argv);
    return 0;
}


