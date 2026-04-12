#include <iostream>
#include <vector>
#include <fstream>
#include <chrono>
#include <sstream>
#include <deque>

#include <opencv2/opencv.hpp>

#include "extract_GPMF.h"
#include "single_digit.h"

using namespace std;
using namespace cv;
using namespace std::chrono;

#ifndef M_PI
#define M_PI 3.14159265
#endif

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


void create_gps_object(gps_data &gps, extracted_data &data, int nb_cols, int nb_rows) {
    //Take the data from the video, check all the gps points, draw a map that contain all these points.
    //It should be a rgba image with white pixel where we have gps.
    //We have to define the orientation (depending on lat/long variation and image size)
    //We ignore edge case when we are close to the poles or crossing 0 lat/long
    //TODO There is a border effect, to be corrected, but nothing that will create bug

    int minx=nb_cols, maxx=0, miny=nb_rows, maxy=0; //used to resize the image.
    gps.cols = nb_cols;
    gps.rows = nb_rows;
    gps.blank_map = cv::Mat::zeros(cv::Size(nb_cols, nb_rows), CV_8UC4);
    gps.road_circle = max(3, max(nb_cols/250, nb_rows/250));
    gps.position_circle = max(5, max(nb_cols/60, nb_rows/60));

    for (auto lat: data.gps_lat) {
        gps.min_lat = min(gps.min_lat, lat);
        gps.max_lat = max(gps.max_lat, lat);
    }
    for (auto lon: data.gps_long) {
        gps.min_long = min(gps.min_long, lon);
        gps.max_long = max(gps.max_long, lon);
    }

    gps.delta_lat = gps.max_lat-gps.min_lat;
    gps.delta_long = gps.max_long-gps.min_long;
    gps.ratio_longlat = gps.delta_long / gps.delta_lat;
    gps.ratio_wh = static_cast<double>(nb_cols)/static_cast<double>(nb_rows);
    if ((gps.ratio_longlat > 1.0 && gps.ratio_wh < 1.0) || (gps.ratio_longlat < 1.0 && gps.ratio_wh > 1.0)) {
        gps.scaling = max(gps.delta_lat/nb_rows, gps.delta_long/nb_cols);
        gps.rotate_image=true;
    }
    else {
        gps.scaling = max(gps.delta_lat/nb_cols, gps.delta_long/nb_rows);;
    }

    cout << "Latitude: [" << gps.min_lat << ", " << gps.max_lat << "]\n";
    cout << "Longitude: [" << gps.min_long << ", " << gps.max_long << "]\n";
    cout << "Deltas: " << gps.delta_lat << ", " << gps.delta_long << "\n";
    cout << "Ratio gps: " << gps.ratio_longlat << "\n";
    cout << "Ratio image: " << gps.ratio_wh << "\n";
    cout << "Scaling: " << gps.scaling << "\n";
    cout << "Rotate? " << gps.rotate_image << "\n";

    for (size_t i = 0; i < data.gps_lat.size(); i++) {
        int x, y;
        if (gps.rotate_image) {
            y = static_cast<int>((data.gps_long[i]-gps.min_long)/gps.scaling);
            x = static_cast<int>((data.gps_lat[i]-gps.min_lat)/gps.scaling);
        }
        else {
            y = static_cast<int>((data.gps_lat[i]-gps.min_lat)/gps.scaling);
            x = static_cast<int>((data.gps_long[i]-gps.min_long)/gps.scaling);
        }
        minx = min(x,minx); maxx = max(x,maxx);
        miny = min(y,miny); maxy = max(y,maxy);
        //gps.blank_map.at<cv::Vec4b>(y,x) = Scalar(255,255,255,255);
        //TODO, I have to do that after I know the minx/maxx to that I have the full map uncut on the borders
        cv::circle(gps.blank_map, cv::Point(x,y), gps.road_circle, cv::Scalar(255,255,255,255),-1);
    }
    gps.blank_map = gps.blank_map(cv::Rect(cv::Point(minx,miny),cv::Point(maxx,maxy))).clone();
    cv::copyMakeBorder(gps.blank_map,gps.blank_map,20,20,20,20,cv::BORDER_CONSTANT, cv::Scalar(0,0,0,0));
    gps.x_shift = 20;
    gps.y_shift = 20;

    cv::rotate(gps.blank_map, gps.blank_map, cv::ROTATE_90_COUNTERCLOCKWISE);

    cv::imwrite("./images/maps.png", gps.blank_map);
}

void display_position_gps(gps_data &gps, extracted_data &data, int &index) {

    if (static_cast<int>(data.gps_lat.size()) < index-1) {
        cerr << "Error index out of range, return" << endl;
        return;
    }
    int x, y;
    //First start from the base image.
    gps.position_map = gps.blank_map.clone();
    //Get the current position on the image
    if (gps.rotate_image) {
        x = static_cast<int>((data.gps_long[index] - gps.min_long) / gps.scaling) + gps.x_shift;
        y = gps.blank_map.rows - static_cast<int>((data.gps_lat[index] - gps.min_lat) / gps.scaling) - gps.y_shift;
    }
    else{
        y = static_cast<int>((data.gps_long[index] - gps.min_long) / gps.scaling) + gps.x_shift;
        x = static_cast<int>((data.gps_lat[index] - gps.min_lat) / gps.scaling) + gps.y_shift;
    }
    cv::circle(gps.position_map, cv::Point(x, y), gps.position_circle, Scalar(255,255,255,255), -1);
    cv::circle(gps.position_map, cv::Point(x, y), 2*gps.position_circle/3, Scalar(255,0,0,255), -1);
}

void read_data_file(const string path_file, extracted_data &data){

    ifstream file(path_file);
    string line;
    while (file >> line){
        //cout << line << endl;
        std::stringstream ss(line);
        std::vector<double> vect;
        for (double i; ss >> i;) {
            vect.push_back(i);
            if (ss.peek() == ',')
                ss.ignore();
        }
        data.gps_lat.emplace_back(vect[0]);
        data.gps_long.emplace_back(vect[1]);
        data.gps_alt.emplace_back(vect[2]);
        data.gps_speed.emplace_back(vect[3]);
        data.gps_speed2.emplace_back(vect[4]);
    }
    cout << "Number of points: " << data.gps_lat.size() << endl;
}

void test_no_video(){

    extracted_data data;
    gps_data gps;
    read_data_file("./data/metadata.txt", data);
    //read_data_file("./data/metadata_sattel.txt", data);
    create_gps_object(gps, data, 1200, 600);

    for (int i = 0; i < (int)data.gps_lat.size(); i++){
        display_position_gps(gps, data, i);
        cv::imshow("overlay", gps.position_map);
        cv::waitKey(10);
    }
}

void fill_digit(cv::Mat &to_fill, int digit, cv::Scalar color){

    static vector<cv::Point> list_centres;
    static vector<vector<bool> > list_segments;
    if (list_segments.empty()){ //Init once
        //Order: top, top left, bottom left, bottom, bottom right, top right, center
        list_segments.push_back({true,true,true,true,true,true,false}); // 0
        list_segments.push_back({false,false,false,false,true,true,false}); // 1
        list_segments.push_back({true,false,true,true,false,true,true}); // 2
        list_segments.push_back({true,false,false,true,true,true,true}); // 3
        list_segments.push_back({false,true,false,false,true,true,true}); // 4
        list_segments.push_back({true,true,false,true,true,false,true}); // 5
        list_segments.push_back({true,true,true,true,true,false,true}); // 6
        list_segments.push_back({true,false,false,false,true,true,false}); // 7
        list_segments.push_back({true,true,true,true,true,true,true}); // 8
        list_segments.push_back({true,true,false,true,true,true,true}); // 9

        list_centres = {
                cv::Point(to_fill.cols/2, to_fill.rows/10),
                cv::Point(to_fill.cols/4, to_fill.rows/3),
                cv::Point(to_fill.cols/4, 2*to_fill.rows/3),
                cv::Point(to_fill.cols/2, 9*to_fill.rows/10),
                cv::Point(3*to_fill.cols/4, 2*to_fill.rows/3),
                cv::Point(3*to_fill.cols/4, to_fill.rows/3),
                cv::Point(to_fill.cols/2, to_fill.rows/2)
        };
    }

    if (digit < 0 || digit > 9) //We only draw single digit
        return;

    for (size_t i = 0; i < list_centres.size(); i++) {
        if (list_segments[digit][i]) cv::floodFill(to_fill, list_centres[i], color);
    }
}

void display_digits(){
    //cv::Mat digit = cv::imread(path_digit);
    cv::Mat digit = get_image_from_h();

    /*for (int i = 0; i < 10; i++){
        cv::Mat curr = digit.clone();
        fill_digit(curr, i);
        cv::imshow("digit", curr);
        cv::waitKey();
    }*/

    //display_speed(digit, 256);

    cv::Mat tacho = drawTachometer(150);
    cv::imshow("tacho", tacho);
    cv::waitKey();
}

cv::Mat display_speed(cv::Mat &digit, uint32_t speed){
    cv::Mat unit = digit.clone(), tens=digit.clone(), hundreds=digit.clone();
    uint32_t _0 = speed%10; speed -= _0; speed/=10;
    uint32_t _00 = speed%10; speed -= _00; speed/=10;
    uint32_t _000 = speed%10;

    fill_digit(unit, _0);
    fill_digit(tens, _00);
    fill_digit(hundreds, _000);
    cv::hconcat(hundreds, tens, hundreds);
    cv::hconcat(hundreds, unit, hundreds);

    cv::imshow("result", hundreds);
    cv::waitKey();

    return hundreds;
}

cv::Mat drawTachometer(int speed) {
    // Constants
    const int width = 600;
    const int height = 600;
    const int radius = 250;
    const cv::Point center(width / 2, height / 2);
    const int minSpeed = 50;
    const int maxSpeed = 300;

    speed = min(maxSpeed, max(minSpeed, speed));

    // Create a blank image with white background
    cv::Mat tachometer(height, width, CV_8UC3, cv::Scalar(255, 255, 255));

    // Draw the outer circle
    cv::circle(tachometer, center, radius, cv::Scalar(0, 0, 0), 2);

    // Draw speed labels and marks
    for (int i = 0; i <= maxSpeed; i += 30) {
        double angle = (i / static_cast<double>(maxSpeed)) * 270.0 - 90;
        double radians = (angle - 135.0) * CV_PI / 180.0;
        int xOuter = static_cast<int>(center.x + radius * cos(radians));
        int yOuter = static_cast<int>(center.y + radius * sin(radians));
        int xInner = static_cast<int>(center.x + (radius - 20) * cos(radians));
        int yInner = static_cast<int>(center.y + (radius - 20) * sin(radians));

        cv::line(tachometer, cv::Point(xOuter, yOuter), cv::Point(xInner, yInner), cv::Scalar(0, 0, 0), 2);

        int labelX = static_cast<int>(center.x + (radius - 40) * cos(radians));
        int labelY = static_cast<int>(center.y + (radius - 40) * sin(radians));
        cv::putText(tachometer, std::to_string(i), cv::Point(labelX - 10, labelY + 5), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 0), 2);
    }

    // Draw the needle
    double speedAngle = (speed / static_cast<double>(maxSpeed)) * 270.0;
    double speedRadians = (speedAngle - 135.0) * CV_PI / 180.0;
    int needleX = static_cast<int>(center.x + (radius - 50) * cos(speedRadians));
    int needleY = static_cast<int>(center.y + (radius - 50) * sin(speedRadians));
    cv::line(tachometer, center, cv::Point(needleX, needleY), cv::Scalar(0, 0, 255), 3);

    // Draw the center circle
    cv::circle(tachometer, center, 10, cv::Scalar(0, 0, 255), -1);

    return tachometer;
}


cv::Mat get_image_from_h(){
    vector<uint8_t> img;
    for (uint32_t i = 0; i < single_digit_png_len; i++){
        img.emplace_back(single_digit_png[i]);
    }
    cv::Mat image;
    image = cv::imdecode(img, cv::IMREAD_COLOR);
    //cv::imshow("image", image);
    //cv::waitKey();
    return image;
}


void read_write_video(){

    auto start = high_resolution_clock::now();
    string video_in = "/home/yohan/Documents/C++/GoProMoto/videos/GH039080.MP4";
    //string video_in = "/home/yohan/Documents/C++/GoProMoto/videos/hero7.MP4";
    string video_out = "/home/yohan/Documents/C++/GoProMoto/videos/GH039080_out.MP4";

    //video_in = "/home/yohan/Documents/gopro/merged/20240718_anneau_1.MP4";
    //video_out = "/home/yohan/Documents/gopro/merged/20240718_anneau_1_mod.MP4";

    //video_in = "./videos/20240716_anneau_1_2.MP4";
    //video_out = "./videos/20240716_anneau_1_2_mod.MP4";

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
    //get_mp4_data(video_in.c_str(), data);
    create_gps_object(gps, data, frame_width/3, frame_height/3);
    convert_ms2kmh(data);


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

int main() {
    //test_no_video();
    display_digits();
    //get_image_from_h();
    return 0;
}
