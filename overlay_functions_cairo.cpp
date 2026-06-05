
#include "overlay_functions_cairo.hpp"

namespace HUD {
    //I need something better than that. It should contains the location of each unit. So that I can configure that easily with a json

    // Speedometer  (bottom-left)
    double SPEED_CX     = 180.0;   // centre X
    double SPEED_CY     = 520.0;   // centre Y
    double SPEED_R      = 140.0;   // outer radius
    double SPEED_MIN    = -220.0;  // angle for 0 km/h   (degrees, 0=right)
    double SPEED_MAX    =  40.0;   // angle for MAX km/h
    double SPEED_KMH_MAX = 280.0;

    // Lean-angle gauge  (bottom-right)
    double LEAN_CX  = 1100.0;
    double LEAN_CY  = 530.0;
    double LEAN_R   = 120.0;
    double LEAN_MAX = 60.0;  // ±60° max lean

    // Track overlay  (Top left)
    double TRACK_CX  = 1000.0;
    double TRACK_CY  = 50.0;
    double TRACK_CW  = 500.0;
    double TRACK_CH  = 500.0;
    double TRACK_CM  = 10.0;
    double TRACK_POS  = 9.0;

    // Lap time (Bottom right) //This one evolve with time
    double LAP_CX  = 1000.0;
    double LAP_CY  = 50.0;
    double LAP_CW  = 500.0;
    double LAP_CH  = 50.0;
    double LAP_CM  = 10.0;

    // Colours
    double BG_ALPHA   = 0.25;   // panel background opacity
    double NEEDLE_R   = 1.0;
    double NEEDLE_G   = 0.18;
    double NEEDLE_B   = 0.18;

}

/// Create an ARGB32 Cairo surface from an BGRA OpenCV Mat.
/// The Mat must already be BGRA (4 channels, CV_8UC4).
cairo_surface_t* mat_to_cairo(const cv::Mat& mat) {
    CV_Assert(mat.type() == CV_8UC4);
    // Cairo uses native-endian ARGB32, OpenCV stores BGRA on little-endian —
    // the byte order happens to match ARGB32 on x86, so we can borrow the data.
    return cairo_image_surface_create_for_data(
        const_cast<unsigned char*>(mat.data),
        CAIRO_FORMAT_ARGB32,
        mat.cols, mat.rows, mat.step[0]);
}

/// Copy a Cairo ARGB32 surface back into a BGRA Mat.
cv::Mat cairo_to_mat(cairo_surface_t* surface) {
    cairo_surface_flush(surface);
    unsigned char* data = cairo_image_surface_get_data(surface);
    int w = cairo_image_surface_get_width(surface);
    int h = cairo_image_surface_get_height(surface);
    int stride = cairo_image_surface_get_stride(surface);
    return cv::Mat(h, w, CV_8UC4, data, stride).clone();
}



/// Draw a rounded rectangle (panel background).
static void rounded_rect(cairo_t* cr, double x, double y, double w, double h, double r) {
    cairo_new_sub_path(cr);
    cairo_arc(cr, x + r,     y + r,     r, deg2rad(180), deg2rad(270));
    cairo_arc(cr, x + w - r, y + r,     r, deg2rad(270), deg2rad(0));
    cairo_arc(cr, x + w - r, y + h - r, r, deg2rad(0),   deg2rad(90));
    cairo_arc(cr, x + r,     y + h - r, r, deg2rad(90),  deg2rad(180));
    cairo_close_path(cr);
}

/// Draw a tick mark at angle θ (degrees) around (cx,cy).
static void tick(cairo_t* cr, double cx, double cy, double r_outer, double r_inner, double theta_deg) {
    double t = deg2rad(theta_deg);
    cairo_move_to(cr, cx + r_outer * cos(t), cy + r_outer * sin(t));
    cairo_line_to(cr, cx + r_inner * cos(t), cy + r_inner * sin(t));
    cairo_stroke(cr);
}

/// Render text centred at (x,y).
static void centred_text(cairo_t* cr, double x, double y, const std::string& s) {
    cairo_text_extents_t ext;
    cairo_text_extents(cr, s.c_str(), &ext);
    cairo_move_to(cr,
                  x - ext.width / 2.0 - ext.x_bearing,
                  y - ext.height / 2.0 - ext.y_bearing);
    cairo_show_text(cr, s.c_str());
}

// ─────────────────────────────────────────────
//  Alpha-composite HUD (BGRA) over a BGR video frame
// ─────────────────────────────────────────────
void overlay_hud(cv::Mat& frame, const cv::Mat& hud_bgra) {
    CV_Assert(frame.type() == CV_8UC3);
    CV_Assert(hud_bgra.type() == CV_8UC4);
    CV_Assert(frame.size() == hud_bgra.size());

    for (int y = 0; y < frame.rows; ++y) {
        const cv::Vec4b* src = hud_bgra.ptr<cv::Vec4b>(y);
        cv::Vec3b*       dst = frame.ptr<cv::Vec3b>(y);
        for (int x = 0; x < frame.cols; ++x) {
            double a = src[x][3] / 255.0;
            if (a < 0.004) continue;
            dst[x][0] = cv::saturate_cast<uchar>(dst[x][0] * (1 - a) + src[x][0] * a);
            dst[x][1] = cv::saturate_cast<uchar>(dst[x][1] * (1 - a) + src[x][1] * a);
            dst[x][2] = cv::saturate_cast<uchar>(dst[x][2] * (1 - a) + src[x][2] * a);
        }
    }
}


void update_HUD_image_size(int width, int height){
    //Put the HUD where I want to according to the image size.
    //Might want to update their size too
    using namespace HUD;   

    SPEED_CX     = 200.0;   // centre X
    SPEED_CY     = height-200.0;   // centre Y
    SPEED_R      = 140.0;   // outer radius
    //SPEED_MIN    = -220.0;  // angle for 0 km/h   (degrees, 0=right)
    //SPEED_MAX    =  40.0;   // angle for MAX km/h
    //SPEED_KMH_MAX = 280.0;

    // Lean-angle gauge  (bottom-right)
    LEAN_CX  = SPEED_CX+300;
    LEAN_CY  =  height-200.0;
    LEAN_R   =  120.0;
    //LEAN_MAX =   60.0;  // ±60° max lean

    // Track overlay  (Top left)
    TRACK_CX  = width-525.0;
    TRACK_CY  = 25.0;
    TRACK_CW  = 500.0;
    TRACK_CH  = 500.0;

    // Lap time (bottom right)
    LAP_CX  = width - 500;
    LAP_CY  = height - 25.0;
    LAP_CW  = 500.0;
    LAP_CH  = 50.0;   

}

cv::Mat create_static_hud(int frame_w, int frame_h){
	cv::Mat canvas = cv::Mat::zeros(frame_h, frame_w, CV_8UC4);
    update_HUD_image_size(frame_w, frame_h);
	return canvas;
}

void create_speed_hud(cv::Mat &canvas){
    //TODO, the rounded rectangle should be partially transparent.
	using namespace HUD;

	cairo_surface_t* surface = mat_to_cairo(canvas);
    cairo_t* cr = cairo_create(surface);

    const double PW = SPEED_R * 2.2, PH = SPEED_R * 2.2;
    const double PX = SPEED_CX - PW / 2.0, PY = SPEED_CY - PH / 2.0;

    // Dark semi-transparent background
    //rounded_rect(cr, PX, PY, PW, PH, 18.0);
    //cairo_set_source_rgba(cr, 0.05, 0.05, 0.08, BG_ALPHA);
    //cairo_fill(cr);

    //Write two arcs, one big black, one small white, more visible
    cairo_set_source_rgba(cr, 0.0, 0.0, 0.0, 1.0);
    cairo_set_line_width(cr, 13.0);
    cairo_arc(cr, SPEED_CX, SPEED_CY, SPEED_R - 10, deg2rad(SPEED_MIN), deg2rad(SPEED_MAX));
    cairo_stroke(cr);
    // Outer arc (grey track)
    cairo_set_source_rgba(cr, 1.0, 1.0, 1.0, 1.0);
    cairo_set_line_width(cr, 6.0);
    cairo_arc(cr, SPEED_CX, SPEED_CY, SPEED_R - 10, deg2rad(SPEED_MIN), deg2rad(SPEED_MAX));
    cairo_stroke(cr);

    // Ticks every 20 km/h, labels every 40
    cairo_set_line_cap(cr, CAIRO_LINE_CAP_ROUND);
    for (int kmh = 0; kmh <= (int)SPEED_KMH_MAX; kmh += 20) {
        double frac  = kmh / SPEED_KMH_MAX;
        double angle = SPEED_MIN + frac * (SPEED_MAX - SPEED_MIN);
        bool major   = (kmh % 40 == 0);

        cairo_set_source_rgba(cr, 1.0, 1.0, 1.0, 1.0);
        cairo_set_line_width(cr, major ? 2.5 : 1.2);
        tick(cr, SPEED_CX, SPEED_CY,  SPEED_R - 12, SPEED_R - (major ? 30.0 : 22.0), angle);

        if (major) {
            double lx = SPEED_CX + (SPEED_R - 48) * cos(deg2rad(angle));
            double ly = SPEED_CY + (SPEED_R - 48) * sin(deg2rad(angle));
            cairo_select_font_face(cr, "sans-serif", CAIRO_FONT_SLANT_NORMAL, CAIRO_FONT_WEIGHT_BOLD);
            cairo_set_font_size(cr, 18.0);
            //Black to create a shadow effect
            cairo_set_source_rgba(cr, 0.0, 0.0, 0.0, 1.0);            
            centred_text(cr, lx+2, ly+2, std::to_string(kmh));

            cairo_set_source_rgba(cr, 0.9, 0.9, 0.9, 1.0);            
            centred_text(cr, lx, ly, std::to_string(kmh));
        }
    }

    // "km/h" label
    cairo_select_font_face(cr, "sans-serif",
                           CAIRO_FONT_SLANT_NORMAL,
                           CAIRO_FONT_WEIGHT_NORMAL);
    cairo_set_font_size(cr, 13.0);
    cairo_set_source_rgba(cr, 0.6, 0.6, 0.6, 1.0);
    centred_text(cr, SPEED_CX, SPEED_CY + 60, "km/h");


    cairo_destroy(cr);
    cairo_surface_destroy(surface);    
}


void create_lean_hud(cv::Mat &canvas){
    //TODO, the rounded rectangle should be partially transparent.

	using namespace HUD;

	cairo_surface_t* surface = mat_to_cairo(canvas);
    cairo_t* cr = cairo_create(surface);

    // ── Lean-angle panel ────────────────────────────────────────────────
    const double PW = LEAN_R * 2.4, PH = LEAN_R * 2.4;
    const double PX = LEAN_CX - PW / 2.0, PY = LEAN_CY - PH / 2.0;

    //rounded_rect(cr, PX, PY, PW, PH, 18.0);
    //cairo_set_source_rgba(cr, 0.05, 0.05, 0.08, BG_ALPHA);
    //cairo_fill(cr);

    // Arc from -LEAN_MAX to +LEAN_MAX mapped to bottom half of circle
    // Convention: 0° lean → pointing down (90°), left lean → <90°, right lean → >90°
    // We map -LEAN_MAX → 90° - LEAN_MAX * scale, etc.
    // Draw grey track arc
    double arc_start = deg2rad(-90.0 - LEAN_MAX);
    double arc_end   = deg2rad(-90.0 + LEAN_MAX);

    //Write two arcs, one big black, one small white, more visible
    cairo_set_source_rgba(cr, 0.0, 0.0, 0.0, 1.0);
    cairo_set_line_width(cr, 13.0);
    cairo_arc(cr, LEAN_CX, LEAN_CY, LEAN_R - 8, arc_start, arc_end);
    cairo_stroke(cr);

    cairo_set_source_rgba(cr, 1.0, 1.0, 1.0, 1.0);
    cairo_set_line_width(cr, 5.0);
    cairo_arc(cr, LEAN_CX, LEAN_CY, LEAN_R - 8, arc_start, arc_end);
    cairo_stroke(cr);

    // Ticks every 10°
    cairo_set_line_cap(cr, CAIRO_LINE_CAP_ROUND);
    for (int lean = -(int)LEAN_MAX; lean <= (int)LEAN_MAX; lean += 10) {
        double angle = -90.0 + lean;   // visual angle (degrees)
        bool major   = (lean % 30 == 0);

        cairo_set_source_rgba(cr, 0.85, 0.85, 0.85, 0.9);
        cairo_set_line_width(cr, major ? 2.5 : 1.2);
        tick(cr, LEAN_CX, LEAN_CY,
             LEAN_R - 10,
             LEAN_R - (major ? 28.0 : 20.0),
             angle);

        if (major && lean != 0) {
            double lx = LEAN_CX + (LEAN_R - 40) * cos(deg2rad(angle));
            double ly = LEAN_CY + (LEAN_R - 40) * sin(deg2rad(angle));

            cairo_select_font_face(cr, "sans-serif", CAIRO_FONT_SLANT_NORMAL, CAIRO_FONT_WEIGHT_BOLD);
            cairo_set_font_size(cr, 14.0);

            //Black to create a shadow effect
            cairo_set_source_rgba(cr, 0.0, 0.0, 0.0, 1.0);            
            centred_text(cr, lx+2, ly+2, (lean > 0 ? "R" : "L") + std::to_string(std::abs(lean)) + "°");

            cairo_set_source_rgba(cr, 0.9, 0.9, 0.9, 1.0);            
            centred_text(cr, lx, ly, (lean > 0 ? "R" : "L") + std::to_string(std::abs(lean)) + "°");
        }
    }

    // Centre vertical reference line (upright)
    cairo_set_source_rgba(cr, 0.5, 0.5, 0.5, 0.6);
    cairo_set_line_width(cr, 1.0);
    cairo_move_to(cr, LEAN_CX, LEAN_CY - LEAN_R + 10);
    cairo_line_to(cr, LEAN_CX, LEAN_CY);
    cairo_stroke(cr);

    // "LEAN" label
    cairo_select_font_face(cr, "sans-serif",
                           CAIRO_FONT_SLANT_NORMAL,
                           CAIRO_FONT_WEIGHT_NORMAL);
    cairo_set_font_size(cr, 13.0);
    cairo_set_source_rgba(cr, 0.6, 0.6, 0.6, 1.0);
    centred_text(cr, LEAN_CX, LEAN_CY + 50, "LEAN");

    cairo_destroy(cr);
    cairo_surface_destroy(surface);    
}


void create_track_hud(cv::Mat &canvas, extracted_data &data, gps_data &gps, laps_data &ld){
	//This is a bit more involved.
	//I need to update the gps_data struct:		
	//  Convert the gps coordinates to image coordinates, include ts
	//  Add finish line and intermediate location in the new coordinate system.
	//  Think on how to display what was riden between two laps.

    using namespace HUD;

    //First, create the list of points for the track to overlay
    // 1. Bounding box in GPS space
    double min_lat = *std::min_element(data.gps_lat.begin(), data.gps_lat.end());
    double max_lat = *std::max_element(data.gps_lat.begin(), data.gps_lat.end());
    double min_lon = *std::min_element(data.gps_long.begin(), data.gps_long.end());
    double max_lon = *std::max_element(data.gps_long.begin(), data.gps_long.end());
    double min_x = DBL_MAX, min_y = DBL_MAX, max_x = 0, max_y = 0;

    // 2. Correct lon scale for latitude (Mercator-lite)
    double lat_mid  = (min_lat + max_lat) / 2.0;
    double cos_lat  = std::cos(lat_mid * M_PI / 180.0);

    double span_lat = max_lat - min_lat;
    double span_lon = (max_lon - min_lon) * cos_lat;  // in "same units" as lat

    // 3. Uniform scale so track fits the box while preserving aspect ratio
    double draw_w = TRACK_CW - 2 * TRACK_CM;
    double draw_h = TRACK_CH - 2 * TRACK_CM;
    double scale  = std::min(draw_w  / span_lon, draw_h  / span_lat);
        
    // 4. Project each point
    gps.track_pts.reserve(data.gps_lat.size());    
    for (size_t i = 0; i < data.gps_lat.size(); ++i) {
        double px = TRACK_CX + TRACK_CM + ((data.gps_long[i] - min_lon) * cos_lat) * scale;
        // Flip Y: screen Y grows downward, lat grows upward
        double py = TRACK_CY + TRACK_CM + (max_lat - data.gps_lat[i]) * scale;
        gps.track_pts.push_back({px, py});        
        //Track the real bounding box
        min_x = min(min_x, px); min_y = min(min_y, py);
        max_x = max(max_x, px); max_y = max(max_y, py);
    }
    // 5. Add the intermediate and finish line in the new coordinate system.
    double px = TRACK_CX + TRACK_CM + ((ld.finish_long - min_lon) * cos_lat) * scale;     
    double py = TRACK_CY + TRACK_CM + (max_lat - ld.finish_lat) * scale;
    gps.track_finish_line = {px, py};
    for (size_t i = 0; i < ld.intermediate_lat.size(); ++i) {
        double px = TRACK_CX + TRACK_CM + ((ld.intermediate_long[i] - min_lon) * cos_lat) * scale;
        // Flip Y: screen Y grows downward, lat grows upward
        double py = TRACK_CY + TRACK_CM + (max_lat - ld.intermediate_lat[i]) * scale;
        gps.track_intermediates.push_back({px, py});        
    }

    //Save the Timestamp
    gps.track_ts = data.gps_ts;
    //Now that we have the basic, we draw the track on the canvas, including the intermediate and the track name

    cairo_surface_t* surface = mat_to_cairo(canvas);
    cairo_t* cr = cairo_create(surface);

    //Using the delta min/max for x and y, update the size of the track HUD
    TRACK_CW = min(TRACK_CW, max_x-min_x+2*TRACK_CM);
    TRACK_CH = min(TRACK_CH, max_y-min_y+2*TRACK_CM);

    //Add the rounded rectangle where the track is
    /*rounded_rect(cr, TRACK_CX, TRACK_CY, TRACK_CW, TRACK_CH, 18.0);
    cairo_set_source_rgba(cr, 0.05, 0.05, 0.08, BG_ALPHA);
    cairo_fill(cr);
    cairo_stroke(cr);*/

    //Draw the track
    if (gps.track_pts.size() >= 2){            
        cairo_new_sub_path(cr);
        cairo_set_line_width(cr, 3.0);
        //Create a shadow effect, draw first black, shifted, then white
        cairo_move_to(cr, gps.track_pts[0].x, gps.track_pts[0].y);
        for (size_t i = 1; i < gps.track_pts.size(); ++i)
            cairo_line_to(cr, gps.track_pts[i].x+2, gps.track_pts[i].y+2);
        cairo_set_source_rgba(cr, 0.0, 0.0, 0.0, 1.0);
        cairo_stroke(cr);   


        cairo_move_to(cr, gps.track_pts[0].x, gps.track_pts[0].y);
        for (size_t i = 1; i < gps.track_pts.size(); ++i)
            cairo_line_to(cr, gps.track_pts[i].x, gps.track_pts[i].y);
        cairo_set_source_rgba(cr, 1.0, 1.0, 1.0, 1.0);        
        cairo_stroke(cr);   
    }
    //Draw the POI
    cairo_move_to(cr, gps.track_finish_line.x, gps.track_finish_line.y);
    cairo_set_source_rgba (cr, 1.0, 1.0, 1.0, 1.0);
    cairo_arc(cr, gps.track_finish_line.x, gps.track_finish_line.y, TRACK_POS+3, 0.0, 2.0*M_PI);
    cairo_fill(cr);
    cairo_set_source_rgba (cr, 0.0, 1.0, 0.0, 0.8);
    cairo_arc(cr, gps.track_finish_line.x, gps.track_finish_line.y, TRACK_POS, 0.0, 2.0*M_PI);
    cairo_fill(cr);

    for (auto pt: gps.track_intermediates) {
        cairo_move_to(cr, pt.x, pt.y);
        cairo_set_source_rgba (cr, 1.0, 1.0, 1.0, 1.0);
        cairo_arc(cr, pt.x, pt.y, TRACK_POS+3, 0.0, 2.0*M_PI);
        cairo_fill(cr);
        cairo_set_source_rgba (cr, 1.0, 1.0, 0.0, 1.0);
        cairo_arc(cr, pt.x, pt.y, TRACK_POS, 0.0, 2.0*M_PI);        
        cairo_fill(cr);
    }
    

    //TODO Add the location of the intermediate points and the finish line.
    //TODO add the track name below and centered.
    

    //Add the track name with shade
    double text_x = TRACK_CX + TRACK_CW/2, text_y = TRACK_CY + TRACK_CH - 2*TRACK_CM;
    
    cairo_select_font_face(cr, "monospace", CAIRO_FONT_SLANT_NORMAL, CAIRO_FONT_WEIGHT_BOLD);    
    cairo_set_font_size(cr, 30.0);
    cairo_set_source_rgba(cr, 0.0, 0.0, 0.0, 1.0);    
    centred_text(cr, text_x + 2, text_y + 2, gps.track_name.c_str());        
    cairo_stroke(cr);
    cairo_set_source_rgba(cr, 1.0, 1.0, 1.0, 1.0);    
    centred_text(cr, text_x, text_y, gps.track_name.c_str());    
    cairo_stroke(cr);

    if (!data.start_ts.empty()){
        cairo_set_font_size(cr, 20.0);
        cairo_set_source_rgba(cr, 0.0, 0.0, 0.0, 1.0);
        centred_text(cr, text_x + 2, text_y + 3*TRACK_CM + 2, data.start_ts.c_str());
        cairo_stroke(cr);
        cairo_set_source_rgba(cr, 1.0, 1.0, 1.0, 1.0);
        centred_text(cr, text_x, text_y + 3*TRACK_CM, data.start_ts.c_str());
        cairo_stroke(cr);
    }    

    cairo_destroy(cr);
    cairo_surface_destroy(surface);

}


// ─────────────────────────────────────────────
//  2. draw_dynamic_hud()
//     Draws onto a COPY of the static layer:
//       • Speed needle + digital readout
//       • Lean-angle needle + digital readout
//
//  speed_kmh : current speed   [0 … SPEED_KMH_MAX]
//  lean_deg  : lean angle       [-LEAN_MAX … +LEAN_MAX]
//              negative = left, positive = right
// ─────────────────────────────────────────────

cv::Mat draw_speed_hud(const cv::Mat& static_layer, double speed_kmh){

    using namespace HUD;

    // Clamp inputs
    speed_kmh = std::clamp(speed_kmh, 0.0, SPEED_KMH_MAX);   
    // Create the cairo objects
    cv::Mat canvas = static_layer.clone();
    cairo_surface_t* surface = mat_to_cairo(canvas);
    cairo_t* cr = cairo_create(surface);

    double frac  = speed_kmh / SPEED_KMH_MAX;
    double angle = SPEED_MIN + frac * (SPEED_MAX - SPEED_MIN);
    double t     = deg2rad(angle);

    // Coloured arc from 0 to current speed
    cairo_set_line_width(cr, 6.0);
    cairo_set_line_cap(cr, CAIRO_LINE_CAP_ROUND);

    // Gradient colour: green → orange → red
    double r = std::min(1.0, frac * 2.0);
    double g = std::min(1.0, 2.0 - frac * 2.0);
    cairo_set_source_rgba(cr, r, g, 0.05, 0.9);

    cairo_arc(cr, SPEED_CX, SPEED_CY, SPEED_R - 10, deg2rad(SPEED_MIN), t);
    cairo_stroke(cr);

    // Needle
    double needle_len = SPEED_R - 20;
    double hub_r      = 10.0;

    cairo_set_source_rgba(cr, NEEDLE_R, NEEDLE_G, NEEDLE_B, 1.0);
    cairo_set_line_width(cr, 3.0);
    cairo_move_to(cr, SPEED_CX - (hub_r + 5) * cos(t), SPEED_CY - (hub_r + 5) * sin(t));
    cairo_line_to(cr, SPEED_CX + needle_len * cos(t),  SPEED_CY + needle_len * sin(t));
    cairo_stroke(cr);

    // Hub cap
    cairo_arc(cr, SPEED_CX, SPEED_CY, hub_r, 0, 2 * M_PI);
    cairo_set_source_rgba(cr, 0.9, 0.9, 0.9, 1.0);
    cairo_fill(cr);

    // Digital speed readout
    char buf[16];
    snprintf(buf, sizeof(buf), "%3.0f", speed_kmh);
    cairo_select_font_face(cr, "monospace", CAIRO_FONT_SLANT_NORMAL, CAIRO_FONT_WEIGHT_BOLD);
    cairo_set_font_size(cr, 40.0);
    cairo_set_source_rgba(cr, 0.0, 0.0, 0.0, 1.0);    
    centred_text(cr, SPEED_CX+2, SPEED_CY+25+2, buf);

    cairo_set_source_rgba(cr, 1.0, 1.0, 1.0, 1.0);    
    centred_text(cr, SPEED_CX, SPEED_CY + 25, buf);

    cairo_destroy(cr);
    cairo_surface_destroy(surface);
    return canvas;
}

cv::Mat draw_lean_hud(const cv::Mat& static_layer, double lean_deg){

    //TODO, to save time, I can add a bool clone, to know if I overwrite the current image or if I must clone it.
    using namespace HUD;
    // Clamp inputs    
    lean_deg  = std::clamp(lean_deg, -LEAN_MAX, LEAN_MAX);

    cv::Mat canvas = static_layer.clone();
    cairo_surface_t* surface = mat_to_cairo(canvas);
    cairo_t* cr = cairo_create(surface);

    double angle = -90.0 + lean_deg;
    double t     = deg2rad(angle);
    double frac  = fabs(lean_deg/LEAN_MAX);

    cairo_new_sub_path(cr);

    // Highlight arc on the active side
    double arc_centre = deg2rad(-90.0);
    cairo_set_line_width(cr, 5.0);
    cairo_set_line_cap(cr, CAIRO_LINE_CAP_ROUND);

    // Colour: white/blue for lean
    //double intensity = std::abs(lean_deg) / LEAN_MAX;
    //cairo_set_source_rgba(cr, 0.2, 0.6 + 0.4 * intensity, 1.0, 0.85);
    double r = std::min(1.0, frac * 2.0);
    double g = std::min(1.0, 2.0 - frac * 2.0);
    cairo_set_source_rgba(cr, r, g, 0.05, 0.9);

    if (lean_deg >= 0) {
        cairo_arc(cr, LEAN_CX, LEAN_CY, LEAN_R - 8, arc_centre, t);
    } else {
        cairo_arc_negative(cr, LEAN_CX, LEAN_CY, LEAN_R - 8, arc_centre, t);
    }
    cairo_stroke(cr);

    // Needle
    double needle_len = LEAN_R - 15;
    double hub_r      = 8.0;
    
    cairo_set_source_rgba(cr, NEEDLE_R, NEEDLE_G, NEEDLE_B, 1.0);
    cairo_set_line_width(cr, 3.0);
    cairo_move_to(cr, LEAN_CX, LEAN_CY);
    cairo_line_to(cr,
                  LEAN_CX + needle_len * cos(t),
                  LEAN_CY + needle_len * sin(t));
    cairo_stroke(cr);
    
    // Hub cap
    cairo_arc(cr, LEAN_CX, LEAN_CY, hub_r, 0, 2 * M_PI);
    cairo_set_source_rgba(cr, 0.9, 0.9, 0.9, 1.0);
    cairo_fill(cr);

    // Digital lean readout
    char buf[16];
    const char* side = (lean_deg < -0.5) ? "L" : (lean_deg >  0.5) ? "R" : " ";
    snprintf(buf, sizeof(buf), "%s%.1f°", side, std::abs(lean_deg));
    cairo_select_font_face(cr, "monospace", CAIRO_FONT_SLANT_NORMAL, CAIRO_FONT_WEIGHT_BOLD);
    cairo_set_font_size(cr, 30.0);

    cairo_set_source_rgba(cr, 0.0, 0.0, 0.0, 1.0);    
    centred_text(cr, LEAN_CX+2, LEAN_CY+20+2, buf);

    cairo_set_source_rgba(cr, 1.0, 1.0, 1.0, 1.0);
    centred_text(cr, LEAN_CX, LEAN_CY+20, buf);

    cairo_destroy(cr);
    cairo_surface_destroy(surface);
    return canvas;

}


cv::Mat draw_track_hud(cv::Mat &static_layer, gps_data &gps, double ts){

    using namespace HUD;
    //Find the idx for this timestamp
    size_t idx = 0;
    while(gps.track_ts[idx] < ts && idx < gps.track_ts.size()-1) idx++;    

    cv::Mat canvas = static_layer.clone();
    cairo_surface_t* surface = mat_to_cairo(canvas);
    cairo_t* cr = cairo_create(surface);

    cairo_move_to(cr, gps.track_pts[idx].x, gps.track_pts[idx].y);

    cairo_set_source_rgba (cr, 0.8, 0.8, 0.8, 1.0);
    cairo_arc(cr, gps.track_pts[idx].x, gps.track_pts[idx].y, TRACK_POS+3, 0.0, 2.0*M_PI);
    cairo_fill(cr);
    cairo_set_source_rgba (cr, 0, 0, 0.8, 1.0);
    cairo_arc(cr, gps.track_pts[idx].x, gps.track_pts[idx].y, TRACK_POS, 0.0, 2.0*M_PI);
    cairo_fill(cr);

    cairo_destroy(cr);
    cairo_surface_destroy(surface);

    return canvas;
}

cv::Mat draw_track_hud_intermediate(cv::Mat &static_layer, gps_data &gps, laps_data &laps, double ts){
    //This one, we draw on top of the track the distance since the last time we passed the finish line.
    //We can have two colors, one since the last intermediate, one for the part of track done
    using namespace HUD;
    cv::Mat canvas = static_layer.clone();
    cairo_surface_t* surface = mat_to_cairo(canvas);
    cairo_t* cr = cairo_create(surface);

    //Get the index of our current position.
    size_t idx = 0;
    while(gps.track_ts[idx] < ts && idx < gps.track_ts.size()-1) idx++; 

    for (auto lap: laps.list_laps){
        if (ts <= lap.list_ts[0] || ts >= lap.list_ts.back()) //Not in this lap.
            continue;

        for (size_t i = 1; i < lap.list_ts.size(); i++){
            //The color depends if we finish this part or not
            if (ts >= lap.list_ts[i]) cairo_set_source_rgba (cr, 1.0, 0.0, 0.0, 1.0);
            else cairo_set_source_rgba (cr, 0.0, 1.0, 0.0, 1.0);

            //Find the location we should start drawing.
            double start_ts = lap.list_ts[i-1], end_ts = min(ts, lap.list_ts[i]);
            size_t idx_start = 0;
            while(gps.track_ts[idx_start] < start_ts && idx_start < gps.track_ts.size()-1) idx_start++; 
            size_t idx_end = idx_start;
            while(gps.track_ts[idx_end] < end_ts && idx_end < gps.track_ts.size()-1) idx_end++; 
            //Now draw over
            if (idx_end-idx_start >= 2){
                cairo_new_sub_path(cr);
                cairo_set_line_width(cr, 5.0);
                cairo_move_to(cr, gps.track_pts[idx_start].x, gps.track_pts[idx_start].y);
                for (size_t i = idx_start; i < idx_end; ++i)
                    cairo_line_to(cr, gps.track_pts[i].x, gps.track_pts[i].y);                
                cairo_stroke(cr);   
            }
        }
    }


    //Now display our position on the track
    cairo_move_to(cr, gps.track_pts[idx].x, gps.track_pts[idx].y);
    cairo_set_source_rgba (cr, 0.8, 0.8, 0.8, 1.0);
    cairo_arc(cr, gps.track_pts[idx].x, gps.track_pts[idx].y, TRACK_POS+3, 0.0, 2.0*M_PI);
    cairo_fill(cr);
    cairo_set_source_rgba (cr, 0, 0, 0.8, 1.0);
    cairo_arc(cr, gps.track_pts[idx].x, gps.track_pts[idx].y, TRACK_POS, 0.0, 2.0*M_PI);
    cairo_fill(cr);

    cairo_destroy(cr);
    cairo_surface_destroy(surface);

    return canvas;


}


cv::Mat draw_track_hud_lean(cv::Mat &static_layer, gps_data &gps, laps_data &laps, lean_data &ld, double ts){
    //This one, we draw on top of the track the lean angle at the different locations.
    //
    using namespace HUD;
    cv::Mat canvas = static_layer.clone();
    cairo_surface_t* surface = mat_to_cairo(canvas);
    cairo_t* cr = cairo_create(surface);

    //Get the index of our current position.
#if 0
    size_t idx = 0;
    while(gps.track_ts[idx] < ts && idx < gps.track_ts.size()-1) idx++; 

    for (auto lap: laps.list_laps){
        if (ts <= lap.list_ts[0] || ts >= lap.list_ts.back()) //Not in this lap.
            continue;

        for (size_t i = 1; i < lap.list_ts.size(); i++){
            //The color depends if we finish this part or not
            if (ts >= lap.list_ts[i]) cairo_set_source_rgba (cr, 1.0, 0.0, 0.0, 1.0);
            else cairo_set_source_rgba (cr, 0.0, 1.0, 0.0, 1.0);

            //Find the location we should start drawing.
            double start_ts = lap.list_ts[i-1], end_ts = min(ts, lap.list_ts[i]);
            size_t idx_start = 0;
            while(gps.track_ts[idx_start] < start_ts && idx_start < gps.track_ts.size()-1) idx_start++; 
            size_t idx_end = idx_start;
            while(gps.track_ts[idx_end] < end_ts && idx_end < gps.track_ts.size()-1) idx_end++; 
            //Now draw over
            if (idx_end-idx_start >= 2){
                cairo_new_sub_path(cr);
                cairo_set_line_width(cr, 5.0);
                cairo_move_to(cr, gps.track_pts[idx_start].x, gps.track_pts[idx_start].y);
                for (size_t i = idx_start+1; i < idx_end; ++i){
                    double frac  = fabs(ld.lean_angle[i]/LEAN_MAX);
                    double r = std::min(1.0, frac * 2.0);
                    double g = std::min(1.0, 2.0 - frac * 2.0);
                    cairo_set_source_rgba(cr, r, g, 0.05, 1.0);
                    cairo_new_path(cr);                    
                    cairo_move_to(cr, gps.track_pts[i-1].x, gps.track_pts[i-1].y);
                    cairo_line_to(cr, gps.track_pts[i].x, gps.track_pts[i].y);
                    cairo_stroke(cr);                    
                }
                //cairo_stroke(cr);   
            }
        }
    }

#endif
    //Instead of just drawing between lap, draw since the last time we were at this point.
    size_t idx = 251, start_idx = 0;
    while(gps.track_ts[idx] < ts && idx < gps.track_ts.size()-1) idx++; 


    for (size_t i = idx-250; i >= 1; --i){ // At 18Hz, 250 > 10s
        //Compute the distance, if below 5 pixels, we stop
        double d = pow(gps.track_pts[i].x - gps.track_pts[idx].x,2) + pow(gps.track_pts[i].y - gps.track_pts[idx].y,2);
        if (d < 25) {
            start_idx = i;
            break;
        }
    }
    //If we start at zero, we have a weird line because of the gps.track_pts[i-1]
    if (start_idx == 0) start_idx++;

    //cairo_new_path(cr);
    cairo_set_line_width(cr, 5.0);
    //cairo_move_to(cr, gps.track_pts[start_idx].x, gps.track_pts[start_idx].y);

    //we draw over several time if needed, if this is to slow, I will need to improve.
    for (size_t i = start_idx; i <= idx; i++){ 
        double frac  = fabs(ld.lean_angle[i]/LEAN_MAX);
        double r = std::min(1.0, frac * 2.0);
        double g = std::min(1.0, 2.0 - frac * 2.0);
        cairo_set_source_rgba(cr, r, g, 0.05, 1.0);
        cairo_new_path(cr);                    
        cairo_move_to(cr, gps.track_pts[i-1].x, gps.track_pts[i-1].y);
        cairo_line_to(cr, gps.track_pts[i].x, gps.track_pts[i].y);
        cairo_stroke(cr);  
    }
    


    //Now display our position on the track
    cairo_move_to(cr, gps.track_pts[idx].x, gps.track_pts[idx].y);
    cairo_set_source_rgba (cr, 0.8, 0.8, 0.8, 1.0);
    cairo_arc(cr, gps.track_pts[idx].x, gps.track_pts[idx].y, TRACK_POS+3, 0.0, 2.0*M_PI);
    cairo_fill(cr);
    cairo_set_source_rgba (cr, 0, 0, 0.8, 1.0);
    cairo_arc(cr, gps.track_pts[idx].x, gps.track_pts[idx].y, TRACK_POS, 0.0, 2.0*M_PI);
    cairo_fill(cr);

    cairo_destroy(cr);
    cairo_surface_destroy(surface);

    return canvas;


}


cv::Mat draw_laptime_hud(cv::Mat &static_layer, laps_data &laps, double start_ts){
    //This one is particular has it has not static HUD background.

    using namespace HUD;

    vector<string> list_display; //One line per lap to display
    vector<cv::Scalar> list_color;
    vector<double> list_lap_time;
    bool finished_lap = false; //We only have a best lap time if we finished at least one lap
    string unfinished = "00:00:00", separator = "|"; //The string for unfinished lap or intermediaire
    
    cv::Scalar c_last(0.0, 0.59, 0.88, 1.0), c_best(0.0, 0.73, 0.46, 1.0), c_done(0.80, 0.45, 0.46, 1.0);
    size_t lap_index = 0;

    for (auto lap: laps.list_laps){
        if (lap.list_ts[0] > start_ts) continue; // We haven't reached that lap yep
        string current=separator;
        lap_index++;
        if (lap.list_ts.back() < start_ts) { // The lap is over
            //Write the lap time
            current += to_string(lap_index) + separator + lap.s_lap_time + separator + separator;
            list_color.emplace_back(c_done);
            list_lap_time.emplace_back(lap.lap_time);
            finished_lap=true;
        }
        else if (lap.list_ts.front() < start_ts){ // The lap started but not over.
            string curr_time = time_to_string(start_ts - lap.list_ts.front());
            current += to_string(lap_index) + separator + curr_time + separator + separator;
            list_color.emplace_back(c_last);
            list_lap_time.emplace_back(DBL_MAX);
        }
        else {
            current += to_string(lap_index) + separator + unfinished + separator + separator;
            list_color.emplace_back(c_last);
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

    //Check if we have something to display
    if (list_display.empty()) return static_layer.clone();

    if (finished_lap){
        double best_time = list_lap_time[0];
        size_t best_index = 0;
        for (size_t i = 0; i < list_lap_time.size(); i++){
            if (best_time > list_lap_time[i]){
                best_time = list_lap_time[i];
                best_index = i;
            }
        }
        list_color[best_index] = c_best;
    }

    //Now display the lap on the bottom right

    cv::Mat canvas = static_layer.clone();
    cairo_surface_t* surface = mat_to_cairo(canvas);
    cairo_t* cr = cairo_create(surface);

    //LAP_CX     LAP_CY     LAP_CW     LAP_CH     LAP_CM
    //This did not seem to have worked.
    rounded_rect(cr, LAP_CX, LAP_CY-LAP_CH*list_display.size()-LAP_CM, LAP_CW, LAP_CH*list_display.size()+2*LAP_CM, 18.0);
    cairo_set_source_rgba(cr, 0.05, 0.05, 0.08, 1.0);
    cairo_fill(cr);


    cairo_select_font_face(cr, "monospace", CAIRO_FONT_SLANT_NORMAL, CAIRO_FONT_WEIGHT_BOLD); //CAIRO_FONT_WEIGHT_NORMAL
    cairo_set_font_size(cr, 40.0);

    for (size_t i = 0; i < list_display.size(); i++){        
        double y = LAP_CY - LAP_CH * (list_display.size()-i);
        
        cairo_set_source_rgba(cr, 0.0, 0.0, 0.0, 1.0);
        centred_text(cr, LAP_CX+2, y+2, list_display[i].c_str());
        
        cairo_set_source_rgba(cr, list_color[i][0], list_color[i][1], list_color[i][2], list_color[i][3]);    
        centred_text(cr, LAP_CX, y, list_display[i].c_str());
    }

    cairo_stroke(cr);

    return canvas;


}