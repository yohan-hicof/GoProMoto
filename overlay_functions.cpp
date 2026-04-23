//
// Created by yohan on 26.07.24.
//

#include "overlay_functions.h"


bool create_gps_object(gps_data &gps, extracted_data &data, int nb_cols, int nb_rows) {
    //Take the data from the video, check all the gps points, draw a map that contain all these points.
    //It should be a rgba image with white pixel where we have gps.
    //We have to define the orientation (depending on lat/long variation and image size)
    //We ignore edge case when we are close to the poles or crossing 0 lat/long
    //We do not clear the vector, since we might want to store several files info in one struct.

    int minx=nb_cols, maxx=0, miny=nb_rows, maxy=0; //used to resize the image.
    gps.cols = nb_cols;
    gps.rows = nb_rows;
    gps.blank_map = cv::Mat::zeros(cv::Size(nb_cols, nb_rows), CV_8UC4);

    gps.road_circle = max(3, max(nb_cols/250, nb_rows/250));
    gps.position_circle = max(5, max(nb_cols/60, nb_rows/60));

    //give default values
    gps.min_lat = gps.min_long = 360;
    gps.max_long = gps.max_lat = -360;

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

    cout << "Min Max Latitude: [" << gps.min_lat << ", " << gps.max_lat << "]\n";
    cout << "Min Max Longitude: [" << gps.min_long << ", " << gps.max_long << "]\n";
    cout << "Deltas: " << gps.delta_lat << ", " << gps.delta_long << "\n";
    cout << "Ratio gps: " << gps.ratio_longlat << "\n";
    cout << "Ratio image: " << gps.ratio_wh << "\n";
    cout << "Scaling: " << gps.scaling << "\n";
    cout << "Rotate? " << (gps.rotate_image ? "True":"False") << "\n";
    //If we add new data, find the previous min/max
    for (auto pt:gps.list_points){
        minx = min(pt.x,minx); maxx = max(pt.x,maxx);
        miny = min(pt.y,miny); maxy = max(pt.y,maxy);
    }

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
        gps.list_points.emplace_back(x,y);
    }


    if (minx == maxx || miny == maxy){
        cerr << "Error while creating the gps object." << endl;
        return false;
    }

    gps.x_shift = 20;
    gps.y_shift = 20;
    //TODO, I have to write the track name here, however, not sure how to do that with the rotation.
    gps.blank_map = gps.blank_map(cv::Rect(cv::Point(minx,miny),cv::Point(maxx,maxy))).clone();
    cv::copyMakeBorder(gps.blank_map,gps.blank_map,gps.y_shift,gps.y_shift,gps.x_shift,
                       gps.x_shift,cv::BORDER_CONSTANT, cv::Scalar(0,0,0,0));

    if (!gps.track_name.empty()){//Add some space at the bottom and write the name of the track
        cv::copyMakeBorder(gps.blank_map,gps.blank_map,0,gps.y_shift,0,
                           0,cv::BORDER_CONSTANT, cv::Scalar(0,0,0,0));
        cv::putText(gps.blank_map, gps.track_name, cv::Point(gps.blank_map.cols/2.5, gps.blank_map.rows-8), cv::FONT_HERSHEY_DUPLEX, 1.5, cv::Scalar(255,255,255,255), 2);
    }
    cv::Point shift(gps.y_shift, gps.y_shift);

    for (auto pt: gps.list_points){
        //pt.y = gps.blank_map.rows - pt.y;
        cv::circle(gps.blank_map, pt+shift, gps.road_circle, cv::Scalar(255,255,255,255),-1);
    }

    if (gps.rotate_image)
        cv::rotate(gps.blank_map, gps.blank_map, cv::ROTATE_90_COUNTERCLOCKWISE);

    //cv::imwrite("./images/maps.png", gps.blank_map);
    return true;
}


void display_position_gps(gps_data &gps, extracted_data &data, double ts) {

    //First start from the base image.
    gps.position_map = gps.blank_map.clone();

    int x, y, index = 0;

    //Find the closest position
    while (data.gps_ts[index] < ts) {
    	index++;
    	if (index >= data.gps_ts.size()-1) break;
    }    
    //Get the current position on the image
    if (gps.rotate_image) {
        x = static_cast<int>((data.gps_long[index] - gps.min_long) / gps.scaling) + gps.x_shift;
        y = gps.blank_map.rows - static_cast<int>((data.gps_lat[index] - gps.min_lat) / gps.scaling) - gps.y_shift;
    }
    else{        
        x = static_cast<int>((data.gps_long[index] - gps.min_long) / gps.scaling) + gps.y_shift;
        y = static_cast<int>((data.gps_lat[index] - gps.min_lat) / gps.scaling) + gps.x_shift;

    }
    cv::circle(gps.position_map, cv::Point(x, y), gps.position_circle, cv::Scalar(255,255,255,255), -1);
    cv::circle(gps.position_map, cv::Point(x, y), 2*gps.position_circle/3, cv::Scalar(255,0,0,255), -1);
}

cv::Mat display_lap_time(laps_data &laps, double start_ts){
    //Take the given image, overlay the laps and intermediate until current index.
    //The image is large with alpha channel.
    cv::Mat overlay = cv::Mat::zeros(1000, 1000, CV_8UC4);
    vector<string> list_display; //One line per lap to display
    vector<cv::Scalar> list_color;
    vector<double> list_lap_time;
    bool finished_lap = false; //We only have a best lap time if we finished at least one lap
    string unfinished = "00:00:00"; //The string for unfinished lap or intermediaire
    string separator = "|";
    cv::Scalar color_last(225, 150, 0, 255), color_best(118, 185, 0, 255), color_done(118, 115, 205, 255);
    size_t lap_index = 0;

    for (auto lap: laps.list_laps){
        if (lap.list_ts[0] > start_ts) continue; // We haven't reached that lap yep
        string current=separator;
        lap_index++;
        if (lap.list_ts.back() < start_ts) { // The lap is over
            //Write the lap time
            current += to_string(lap_index) + separator + lap.s_lap_time + separator + separator;
            list_color.emplace_back(color_done);
            list_lap_time.emplace_back(lap.lap_time);
            finished_lap=true;
        }
        else if (lap.list_ts.front() < start_ts){ // The lap started but not over.
            string curr_time = time_to_string(start_ts - lap.list_ts.front());
            current += to_string(lap_index) + separator + curr_time + separator + separator;
            list_color.emplace_back(color_last);
            list_lap_time.emplace_back(DBL_MAX);
        }
        else {
            current += to_string(lap_index) + separator + unfinished + separator + separator;
            list_color.emplace_back(color_last);
            list_lap_time.emplace_back(DBL_MAX);
        }
        //TODO once it is tested for general lap time, display the real time counter for intermediaire times.
        for (size_t i = 1; i < lap.list_ts.size(); i++){
            if (lap.list_ts[i] < start_ts)// We reached this intermediate.
                current += lap.s_intermediate_time[i-1] + separator;
            else if(lap.list_ts[i-1] < start_ts) {// We started this section, but not finished yet
                string curr_time = time_to_string(start_ts - lap.list_ts[i-1]);                
                current += curr_time + separator;
            }
            else
                current += unfinished + separator;
        }
        list_display.emplace_back(current);
    }
    //Find the best lap_time and update its color
    if (finished_lap){
        double best_time = list_lap_time[0];
        size_t best_index = 0;
        for (size_t i = 0; i < list_lap_time.size(); i++){
            if (best_time > list_lap_time[i]){
                best_time = list_lap_time[i];
                best_index = i;
            }
        }
        list_color[best_index] = color_best;
    }
    //Put the text onto the image.
    for (size_t i = 0; i < list_display.size(); i++){
        cv::putText(overlay, list_display[i], cv::Point(5, 50*i+50), cv::FONT_HERSHEY_DUPLEX, 1.5, list_color[i], 2);
    }
    //Crop the image to its minimal size
    overlay = overlay(cv::Rect(cv::Point(0,0), cv::Point(overlay.cols, 50*list_display.size()+25))).clone();

    //cv::imshow("image", overlay);
    //cv::waitKey();
    return overlay;

}

void crop_borders(cv::Mat &image){
    //Take an image, crop all the areas with no pixels
    for (int y = 0; y < image.rows; y++){
        bool found = false;
        int last_x = 0;
        for (int x = 0; x < image.cols; x++){
            if (image.at<cv::Vec4b>(y,x)[0] != 0 || image.at<cv::Vec4b>(y,x)[1] != 0 || image.at<cv::Vec4b>(y,x)[2] != 0) {
                found = true;
                last_x = x;
            }
        }
    }

}

void create_speed_meter(speed_overlay &speed, const size_t width, const size_t height){

    speed.cols = width;
    speed.rows = height;
    speed.blank_speed = cv::Mat::zeros(width, height, CV_8UC4);

    speed.centre = cv::Point(width/2, height/2);
    speed.radius = 0.9 * min(width/2, height/2);
    speed.thickness = max(1.0, 0.1 * speed.radius);
    cv::Scalar slow(0,225,0,255), mid(0, 200, 233, 255), fast(0,0,225,255);
    //The min and max speed and not correctly named. We want speed between 60 and 300, but here we add the rotation.
    double min_speed = 150.0, max_speed = 390.0;

    for (double angle = min_speed; angle <= max_speed; angle += 0.01){
        cv::Point2f p1, p2;
        double rad = angle*M_PI/180.0;
        p2.x = speed.radius * cos(rad) + speed.centre.x;
        p2.y = speed.radius * sin(rad) + speed.centre.y;
        p1.x = 0.85*speed.radius * cos(rad) + speed.centre.x;
        p1.y = 0.85*speed.radius * sin(rad) + speed.centre.y;
        double r = (angle-min_speed)/(max_speed- min_speed);
        cv::Scalar color = r*fast + (1.0-r)*slow;
        cv::line(speed.blank_speed, p1, p2, color);
    }
    //Transform the black into transparent
    cv::Mat mask;
    inRange(speed.blank_speed, cv::Scalar(0, 0, 0, 0), cv::Scalar(10, 10, 10, 255), mask);
    speed.blank_speed.setTo(cv::Scalar(0, 0, 0, 0), mask);

    //Crop the useless borders


    //Create the digits images
    cv::Mat blank_digit = get_digit_image_from_h();

    double r = 0.15*width/blank_digit.cols;
    cv::resize(blank_digit, blank_digit, cv::Size(0,0), r, r);
    for (int i = 0; i < 10; i++){
        cv::Mat curr_digit = blank_digit.clone();
        // fill_digit(curr_digit, i, cv::Scalar(23,101,232, 255));
        fill_digit(curr_digit, i, cv::Scalar(5,10,200, 255));
        cv::cvtColor(curr_digit, curr_digit, cv::COLOR_BGR2BGRA); //Cannot do that before because floodfill
        speed.list_digits.emplace_back(curr_digit);
    }
}

void display_speed(speed_overlay &speed, extracted_data &data, double ts){
    //Find the correct speed using the extracted data and the timestamp, then call the function with the speed.
    //This is an update of the previous function that was using the imprecise index.
    int index = 0;
    while(data.gps_ts[index] < ts){
        index++;
        if (index >= data.gps_ts.size()-1) break;
    }
    double s = data.gps_speed[index];
    display_speed(speed, s);
}

void display_speed(speed_overlay &speed, double v){
    //Overlay the needle on top of an existing blank speed-meter
    //Later also display the digits.

    speed.curr_speed = speed.blank_speed.clone();

    //Adjust V between 60-300
    double adjusted_v = max(min(300.0,v), 60.0);
    cv::Point2f p1, p2;
    double rad = (adjusted_v+90)*M_PI/180.0;
    p2.x = speed.radius * cos(rad) + speed.centre.x;
    p2.y = speed.radius * sin(rad) + speed.centre.y;
    p1.x = 0.60*speed.radius * cos(rad) + speed.centre.x;
    p1.y = 0.60*speed.radius * sin(rad) + speed.centre.y;

    //Add the line for the speed
    cv::Scalar color(255,255,255,255), color_black(0,0,0,255);
    int thickness_out = max(7, min(speed.curr_speed.rows, speed.curr_speed.cols)/100);
    int thickness_in = thickness_out-4;
    cv::line(speed.curr_speed, p1, p2, color, thickness_out);
    cv::line(speed.curr_speed, p1, p2, color_black, thickness_in);

    //Create the image with the 3 digits
    cv::Mat digits;
    int int_speed = static_cast<int>(v);
    int_speed = min(max(int_speed, 0), 400);
    uint32_t _0 = int_speed%10; int_speed -= _0; int_speed/=10;
    uint32_t _00 = int_speed%10; int_speed -= _00; int_speed/=10;
    uint32_t _000 = int_speed%10;

    cv::hconcat(speed.list_digits[_000], speed.list_digits[_00], digits);
    cv::hconcat(digits, speed.list_digits[_0], digits);
    //Overlay the speed on top of the speedmeter
    cv::Point tl((speed.curr_speed.cols-digits.cols)/2, 0.6*speed.curr_speed.rows- digits.rows);
    digits.copyTo(speed.curr_speed(cv::Rect(tl, digits.size())));
}


cv::Mat get_digit_image_from_h(){
    vector<uint8_t> img;
    for (uint32_t i = 0; i < single_digit_png_len; i++){
        img.emplace_back(single_digit_png[i]);
    }
    cv::Mat image;
    image = cv::imdecode(img, cv::IMREAD_COLOR);
    return image;
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
