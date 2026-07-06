//
// Created by yohan on 6/13/24.
//
/*! @file GPMF_demo.c
 *
 *  @brief Demo to extract GPMF from an MP4
 *
 *  @version 2.5.0
 *
 *  (C) Copyright 2017-2020 GoPro Inc (http://gopro.com/).
 *
 *  Licensed under either:
 *  - Apache License, Version 2.0, http://www.apache.org/licenses/LICENSE-2.0
 *  - MIT license, http://opensource.org/licenses/MIT
 *  at your option.
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 */

#include "extract_GPMF.h"

uint32_t show_computed_samplerates = SHOW_COMPUTED_SAMPLERATES;
int fuzzloopcount = 0;


int get_mp4_data(const char* filename, extracted_data &data) {

	GPMF_ERR ret = GPMF_OK;
    uint32_t old_nb_frames = data.nb_frames;

    GetStartTimestampFromMP4(filename, data);

    do{
	ret = GetGPSMP4File(filename, data);
    } while (ret == GPMF_OK && --fuzzloopcount > 0);

    //Correct the first data points (no gps yet). Then check for the consistency
    correct_gps_data(data);
    check_gps_data(data);

    do{
	ret = GetACCLMP4File(filename, data);
    } while (ret == GPMF_OK && --fuzzloopcount > 0);

    do{
	ret = GetGYROMP4File(filename, data);
    } while (ret == GPMF_OK && --fuzzloopcount > 0);

    //average_imu_data(data, 51);

    //Update the number of frames in case of several videos files
    data.nb_frames += old_nb_frames;

#if 0
	for (size_t i = 0; i < data.gps_alt.size(); i++) {
		cerr << "Altitude: " << data.gps_alt[i] << data.gps_alt_unit << ", Latitude" << data.gps_lat[i];
		cerr << data.gps_lat_unit << ", Longitude" << data.gps_long[i] << data.gps_long_unit << endl;
	}
	for (size_t i = 0; i < data.accl_x.size(); i++) {
		cerr << "ACCL x:" << data.accl_x[i] << data.accl_x_unit << ", y:" << data.accl_y[i];
		cerr << data.accl_y_unit << ", z:" << data.accl_z[i] << data.accl_z_unit << endl;
	}

	for (size_t i = 0; i < data.gyro_x.size(); i++) {
		cerr << "GYRO x:" << data.gyro_x[i] << data.gyro_x_unit << ", y:" << data.gyro_y[i];
		cerr << data.gyro_y_unit << ", z:" << data.gyro_z[i] << data.gyro_z_unit << endl;
	}
#endif
	return 0;

}


void correct_gps_data(extracted_data &data){
    //The GPS data are all 0 before the GPS aquire the signal.
    //We find the first non zero value and copy it to the previous null values

    size_t pos = 0;
    for (size_t i = 0; i < data.gps_lat.size(); i++){
        if (fabs(data.gps_lat[i]) < 0.01 && fabs(data.gps_long[i]) < 0.01 && fabs(data.gps_alt[i]) < 0.01)
            continue;
        pos = i;
        break;
    }
    for (size_t i = 0; i < pos; i++){
        data.gps_lat[i] = data.gps_lat[pos];
        data.gps_long[i] = data.gps_long[pos];
        data.gps_alt[i] = data.gps_alt[pos];
        data.gps_speed[i] = data.gps_speed[pos];
        data.gps_speed2[i] = data.gps_speed2[pos];
    }


    //In some case the GPS goes nuts and give me values Kms away for a couple of seconds.
    // This correct the issue by getting the mean lat/long values and keeping the more likely once we detect a jump.
    //Note that I check also backward, but it might not be necessary. Since it takes about 2ms for a 20 mins video
    // We do not care.
    double mean_lat = 0.0, mean_long = 0.0;
    vector<double> list_dist;
    for (auto l: data.gps_lat) mean_lat += l;
    for (auto l: data.gps_long) mean_long += l;
    mean_lat /= data.gps_lat.size();
    mean_long /= data.gps_long.size();

    for (size_t i = 1; i < data.gps_lat.size(); i++)
        list_dist.emplace_back(haversine_meters(data.gps_lat[i-1], data.gps_long[i-1],data.gps_lat[i], data.gps_long[i]));

    for (size_t i = 1; i < data.gps_lat.size(); i++){
        double dist = haversine_meters(data.gps_lat[i-1], data.gps_long[i-1],data.gps_lat[i], data.gps_long[i]);
        if (dist > 2000){// We have a jump of more than 2km. At 18Hz that would mean 36km/s, not my cbr
            double dist1 = haversine_meters(data.gps_lat[i-1], data.gps_long[i-1],mean_lat, mean_long);
            double dist2 = haversine_meters(data.gps_lat[i], data.gps_long[1],mean_lat, mean_long);
            if (dist1 < dist2) {//i.e. the point i is the issue.
                data.gps_lat[i] = data.gps_lat[i-1];
                data.gps_long[i] = data.gps_long[i-1];
            }
        }
    }
    //Now the same backward
    for (size_t i = data.gps_lat.size()-1; i > 0; i--){
        double dist = haversine_meters(data.gps_lat[i-1], data.gps_long[i-1],data.gps_lat[i], data.gps_long[i]);
        if (dist > 2000){// We have a jump of more than 2km. At 18Hz that would mean 36km/s, not my cbr
            double dist1 = haversine_meters(data.gps_lat[i-1], data.gps_long[i-1],mean_lat, mean_long);
            double dist2 = haversine_meters(data.gps_lat[i], data.gps_long[1],mean_lat, mean_long);
            if (dist1 > dist2) {//i.e. the point i is the issue.
                data.gps_lat[i-1] = data.gps_lat[i];
                data.gps_long[i-1] = data.gps_long[i];
            }
        }
    }

}

bool check_gps_data(extracted_data &data){
    //The idea is to check if consecutive points in the gps data are close. i.e. if d(pt[i],pt[i+1]) < epsilon
    //With epsilon a small distance like 10 meters.
    //Assuming 8 points per second for the gps, max speed 300km/h -> 83m/s -> 10.4m/point
    
    double epsilon = 25;
    int nb_jumps = 0;
    
    for (size_t i = 1; i < data.gps_lat.size(); i++){
        double distance = haversine_meters(data.gps_lat[i], data.gps_long[i], data.gps_lat[i-1], data.gps_long[i-1]);
        if (distance > epsilon) nb_jumps++;    
    }
    if (nb_jumps != 0){
        cerr << "Error in GPS data, we have " << nb_jumps << " jumps in the data" << endl;
        return false;
    }
    return true;
}

std::vector<double> gaussian_smooth(const std::vector<double>& v, int ksize) {
    int half = ksize / 2;
    double sigma = ksize / 3.0;  // reasonable default

    // build kernel
    std::vector<double> kernel(ksize);
    double sum = 0.0;
    for (int i = 0; i < ksize; ++i) {
        int x = i - half;
        kernel[i] = std::exp(-(x * x) / (2 * sigma * sigma));
        sum += kernel[i];
    }
    for (double& w : kernel) w /= sum;

    // convolve
    std::vector<double> out(v.size());
    for (size_t i = 0; i < v.size(); ++i) {
        double acc = 0.0;
        for (int j = 0; j < ksize; ++j) {
            int idx = int(i) + j - half;
            // clamp edges
            if (idx < 0) idx = 0;
            if (idx >= (int)v.size()) idx = v.size() - 1;
            acc += v[idx] * kernel[j];
        }
        out[i] = acc;
    }
    return out;
}

void average_imu_data(extracted_data &data, int window_size){
    //Average the accl and gyro data over a window. The window should be an odd value
    //Use the following coefficient [1/k,2/k,..., 1 ,k-1/k,k-2/k,...1/k] with k = floor(window_size/2)
    
    if (window_size <= 0 || window_size%2 == 0){
        cerr << "Error in " << __FUNCTION__ << " invalid window size" << endl;
        return;
    }
    
    if (data.accl_x.size() == 0 || data.gyro_x.size() == 0){
    	cerr << "Error in " << __FUNCTION__ << " empty input vector" << endl;
        return;
    }
    
    data.accl_x = gaussian_smooth(data.accl_x, window_size);
    data.accl_y = gaussian_smooth(data.accl_y, window_size);
    data.accl_z = gaussian_smooth(data.accl_z, window_size);
    data.gyro_x = gaussian_smooth(data.gyro_x, window_size);
    data.gyro_y = gaussian_smooth(data.gyro_y, window_size);
    data.gyro_z = gaussian_smooth(data.gyro_z, window_size);      
    
    return;          
    
    int half_window = window_size/2;
    vector<double> coefs(half_window,1.0);
    double sum_coef = 1.0;
    
    for (int i = 0; i < half_window; i++){
    	coefs[i] = static_cast<double>(i)/static_cast<double>(half_window);
    	sum_coef += 2*coefs[i];
    }
    
    //Create copy of the vectors to smooth
    std::vector<double> accl_x=data.accl_x, accl_y=data.accl_y, accl_z=data.accl_z;
    std::vector<double> gyro_x=data.gyro_x, gyro_y=data.gyro_y, gyro_z=data.gyro_z;   
    
    for (size_t i = half_window+1; i < accl_x.size()-half_window-1; i++){
        double ax = accl_x[i+half_window], ay = accl_y[i+half_window], az = accl_z[i+half_window];
        double gx = gyro_x[i+half_window], gy = gyro_y[i+half_window], gz = gyro_z[i+half_window];
        for (size_t j = 0; j < half_window; j++){            
            ax += (accl_x[i+half_window+j]+accl_x[i+j])*coefs[j];
            ay += (accl_y[i+half_window+j]+accl_y[i+j])*coefs[j];
            az += (accl_z[i+half_window+j]+accl_z[i+j])*coefs[j];
            
            gx += (gyro_x[i+half_window+j]+gyro_x[i+j])*coefs[j];
            gy += (gyro_y[i+half_window+j]+gyro_y[i+j])*coefs[j];
            gz += (gyro_z[i+half_window+j]+gyro_z[i+j])*coefs[j];        
        }
        ax /= sum_coef; ay /= sum_coef; az /= sum_coef;
        gx /= sum_coef; gy /= sum_coef; gz /= sum_coef;
    	//Update the value on the final vector
    	data.accl_x[i+half_window] = ax; data.accl_y[i+half_window] = ay; data.accl_z[i+half_window] = az;
        data.gyro_x[i+half_window] = gx; data.gyro_y[i+half_window] = gy; data.gyro_z[i+half_window] = gz;    	
    }   


}

void write_mp4_metadata(const string filename, extracted_data &data){

    ofstream file(filename);

    file << "Framerate: " << data.framerate << "\n";
    file << "Nb Frames: " << data.nb_frames << "\n";

    file << "---GPS---\n";
    file << "GPS start: " << data.gps_start << "\n";
    file << "GPS end: " << data.gps_end << "\n";
    file << "GPS lat, long, alt, speed, speed2: " << "\n";
    for (size_t i = 0; i < data.gps_lat.size(); i++){
        file << data.gps_lat[i] <<","<< data.gps_long[i] <<","<< data.gps_alt[i] <<",";
        file << data.gps_speed[i] <<","<< data.gps_speed2[i] <<"\n";

    }
    //TODO The same with the other data when I need to debug them.
    file.close();

}

void write_mp4_all_metadata(const string filename, extracted_data &data, size_t nb_data_points){

    // Add all the information, not just these one.
    if (nb_data_points <= 0) nb_data_points = data.gps_lat.size();

    ofstream file(filename);

    //First show all the important values and units
    file << "Framerate,Nb Frames\n"; 
    file << data.framerate << "," << data.nb_frames << "\n"; 
    file << "---GPS---,start,end,lat unit,lon unit,alt unit,speed unit,speed2 unit,rate\n,";
    file << data.gps_start << "," << data.gps_end;
    file << "," << data.gps_lat_unit << "," << data.gps_long_unit << "," << data.gps_alt_unit;
    file << "," << data.gps_speed_unit << "," << data.gps_speed2_unit << "," << data.gpsrate << "\n";

    file << "---ACCl---,start,end,X unit,Y unit,Z unit,rate\n,";
    file << data.accl_start << "," << data.accl_end;
    file << "," << data.accl_x_unit << "," << data.accl_y_unit << "," << data.accl_z_unit;
    file << "," << data.acclrate << "\n";
    
    file << "---GYRO---,start,end,X unit,Y unit,Z unit,rate\n,";
    file << data.gyro_start << "," << data.gyro_end;
    file << "," << data.gyro_x_unit << "," << data.gyro_y_unit << "," << data.gyro_z_unit;
    file << "," << data.gyrorate << "\n";

    //Now the data themselves
    file << "GPS lat, long, alt, speed, speed2, GPS ts, accl_x, accl_y, accl_z, gyro_x, gyro_y, gyro_z" << "\n";
    for (size_t i = 0; i < nb_data_points; i++){
        file << std::setprecision(6) << data.gps_lat[i] << "," <<  std::setprecision(6) << data.gps_long[i] <<","<<  std::setprecision(6) << data.gps_alt[i] <<",";
        file << data.gps_speed[i] <<","<< data.gps_speed2[i] << ","  << std::setprecision(6) << data.gps_ts[i];
        file << "," << data.accl_x[i] << "," << data.accl_y[i] << "," << data.accl_z[i];
        file << "," << data.gyro_x[i] << "," << data.gyro_y[i] << "," << data.gyro_z[i] <<"\n";
    }
    //TODO The same with the other data when I need to debug them.
    file.close();

}

GPMF_ERR GetStartTimestampFromMP4(const char* filename, extracted_data& data){

    size_t mp4handle = OpenMP4Source(filename, MOV_GPMF_TRAK_TYPE, MOV_GPMF_TRAK_SUBTYPE, 0);
    if (mp4handle == 0) {
        printf("error: %s is an invalid MP4/MOV or it has no GPMF data\n\n", filename);
        return GPMF_ERROR_BAD_STRUCTURE;
    }

    GPMF_ERR ret = GPMF_ERROR_FIND;

    uint32_t payloads = GetNumberPayloads(mp4handle);
    size_t payloadres = 0;

    for (uint32_t index = 0; index < payloads; index++) {
        uint32_t payloadsize = GetPayloadSize(mp4handle, index);
        payloadres = GetPayloadResource(mp4handle, payloadres, payloadsize);
        uint32_t* payload = GetPayload(mp4handle, payloadres, index);
        if (payload == NULL)
            break;

        GPMF_stream ms = { 0 };
        if (GPMF_Init(&ms, payload, payloadsize) != GPMF_OK)
            continue;

        if (GPMF_OK == GPMF_FindNext(&ms, STR2FOURCC("GPSU"),
                static_cast<GPMF_LEVELS>(GPMF_RECURSE_LEVELS | GPMF_TOLERANT)))
        {
            uint32_t len = GPMF_RawDataSize(&ms);
            char buf[32] = {};
            memcpy(buf, GPMF_RawData(&ms), std::min(len, (uint32_t)sizeof(buf) - 1));

            // GPSU format: "YYMMDDHHMMSS.sss"
            int YY, MM, DD, hh, mm, ss;
            if (sscanf(buf, "%2d%2d%2d%2d%2d%2d", &YY, &MM, &DD, &hh, &mm, &ss) == 6) {
                char result[32];
                snprintf(result, sizeof(result), "20%02d-%02d-%02d %02d:%02d:%02d UTC",
                         YY, MM, DD, hh, mm, ss);
                data.start_ts = result;
                ret = GPMF_OK;
            }
            GPMF_Free(&ms);
            break; // Found what we need, stop
        }

        GPMF_Free(&ms);
    }

    if (payloadres) FreePayloadResource(mp4handle, payloadres);
    CloseSource(mp4handle);

    return ret;
}


GPMF_ERR GetGPSMP4File(const char* filename, extracted_data &data){
	GPMF_ERR ret = GPMF_OK;
	GPMF_stream metadata_stream = { 0 }, * ms = &metadata_stream;
	double metadatalength;
	uint32_t* payload = NULL;
	uint32_t payloadsize = 0;
	size_t payloadres = 0;
	size_t mp4handle = OpenMP4Source(filename, MOV_GPMF_TRAK_TYPE, MOV_GPMF_TRAK_SUBTYPE, 0);
    double last_ts = 0.0; 
    if (data.gps_ts.size() != 0) last_ts = data.gps_ts.back();

	if (mp4handle == 0){
		printf("error: %s is an invalid MP4/MOV or it has no GPMF data\n\n", filename);
		return GPMF_ERROR_BAD_STRUCTURE;
	}

	metadatalength = GetDuration(mp4handle);

	if (metadatalength > 0.0){
		uint32_t index, payloads = GetNumberPayloads(mp4handle);

		uint32_t fr_num, fr_dem;
		uint32_t frames = GetVideoFrameRateAndCount(mp4handle, &fr_num, &fr_dem);
		data.nb_frames = frames;
		data.framerate = (double)fr_num/(double)fr_dem;


		for (index = 0; index < payloads; index++){
			double in = 0.0, out = 0.0; //times
			payloadsize = GetPayloadSize(mp4handle, index);
			payloadres = GetPayloadResource(mp4handle, payloadres, payloadsize);
			payload = GetPayload(mp4handle, payloadres, index);
			if (payload == NULL)
				goto cleanup;

			ret = GetPayloadTime(mp4handle, index, &in, &out);
			if (ret != GPMF_OK)
				goto cleanup;

			ret = GPMF_Init(ms, payload, payloadsize);
			if (ret != GPMF_OK)
				goto cleanup;

			while (GPMF_OK == GPMF_FindNext(ms, STR2FOURCC("STRM"), static_cast<GPMF_LEVELS>(GPMF_RECURSE_LEVELS|GPMF_TOLERANT))) //GoPro Hero5/6/7 Accelerometer)
			{

				if (GPMF_OK != GPMF_FindNext(ms, STR2FOURCC("GPS5"), static_cast<GPMF_LEVELS>(GPMF_RECURSE_LEVELS|GPMF_TOLERANT)))
						continue;

				uint32_t samples = GPMF_Repeat(ms);
				uint32_t elements = GPMF_ElementsInStruct(ms);

				uint32_t buffersize = samples * elements * sizeof(double);
				GPMF_stream find_stream;
				double* ptr, * tmpbuffer = (double*)malloc(buffersize);

				#define MAX_UNITS	64
				#define MAX_UNITLEN	8
				char units[MAX_UNITS][MAX_UNITLEN] = { "" };
				uint32_t unit_samples = 1;

				char complextype[MAX_UNITS] = { "" };
				uint32_t type_samples = 1;

				if (tmpbuffer)
				{
					uint32_t i;
					//Search for any units to display
					GPMF_CopyState(ms, &find_stream);
					if (GPMF_OK == GPMF_FindPrev(&find_stream, GPMF_KEY_SI_UNITS, static_cast<GPMF_LEVELS>(GPMF_CURRENT_LEVEL | GPMF_TOLERANT)) ||
						GPMF_OK == GPMF_FindPrev(&find_stream, GPMF_KEY_UNITS, static_cast<GPMF_LEVELS>(GPMF_CURRENT_LEVEL | GPMF_TOLERANT)))
					{
						char* data = (char*)GPMF_RawData(&find_stream);
						uint32_t ssize = GPMF_StructSize(&find_stream);
						if (ssize > MAX_UNITLEN - 1) ssize = MAX_UNITLEN - 1;
						unit_samples = GPMF_Repeat(&find_stream);

						for (i = 0; i < unit_samples && i < MAX_UNITS; i++)
						{
							memcpy(units[i], data, ssize);
							units[i][ssize] = 0;
							data += ssize;
						}
					}
					if (data.gps_alt_unit.empty()) {
						data.gps_lat_unit = units[0];
						data.gps_long_unit = units[1];
						data.gps_alt_unit = units[2];
						data.gps_speed_unit = units[3];
						data.gps_speed2_unit = units[4];
					}
					//Search for TYPE if Complex
					GPMF_CopyState(ms, &find_stream);
					type_samples = 0;
					if (GPMF_OK == GPMF_FindPrev(&find_stream, GPMF_KEY_TYPE, static_cast<GPMF_LEVELS>(GPMF_CURRENT_LEVEL | GPMF_TOLERANT)))
					{
						char* data = (char*)GPMF_RawData(&find_stream);
						uint32_t ssize = GPMF_StructSize(&find_stream);
						if (ssize > MAX_UNITLEN - 1) ssize = MAX_UNITLEN - 1;
						type_samples = GPMF_Repeat(&find_stream);

						for (i = 0; i < type_samples && i < MAX_UNITS; i++)
						{
							complextype[i] = data[i];
						}
					}

					if (GPMF_OK == GPMF_ScaledData(ms, tmpbuffer, buffersize, 0, samples, GPMF_TYPE_DOUBLE)){//Output scaled data as floats
						ptr = tmpbuffer;
						int pos = 0;
						for (i = 0; i < samples; i++)	{
							data.gps_lat.emplace_back(*ptr++);
							data.gps_long.emplace_back(*ptr++);
							data.gps_alt.emplace_back(*ptr++);
							data.gps_speed.emplace_back(*ptr++);
							data.gps_speed2.emplace_back(*ptr++);
							
							data.gps_ts.emplace_back(in+i*(out-in)/samples + last_ts);
                            
							/*printf("--%.3f%s, ", data.gps_lat.back(), units[0 % unit_samples]);
							printf("--%.3f%s, ", data.gps_long.back(), units[1 % unit_samples]);
							printf("--%.3f%s, ", data.gps_alt.back(), units[2 % unit_samples]);
							printf("--%.3f%s, ", data.gps_speed.back(), units[3 % unit_samples]);
							printf("--%.3f%s ", data.gps_speed2.back(), units[4 % unit_samples]);
							if (fuzzloopcount == 0) printf("\n");*/
						}
					}
					free(tmpbuffer);
				}

			}
			GPMF_ResetState(ms);

		}


		if (show_computed_samplerates)
		{
			mp4callbacks cbobject;
			cbobject.mp4handle = mp4handle;
			cbobject.cbGetNumberPayloads = GetNumberPayloads;
			cbobject.cbGetPayload = GetPayload;
			cbobject.cbGetPayloadSize = GetPayloadSize;
			cbobject.cbGetPayloadResource = GetPayloadResource;
			cbobject.cbGetPayloadTime = GetPayloadTime;
			cbobject.cbFreePayloadResource = FreePayloadResource;
			cbobject.cbGetEditListOffsetRationalTime = GetEditListOffsetRationalTime;

			double start, end;
			double rate_GPS = GetGPMFSampleRate(cbobject, STR2FOURCC("GPS5"), STR2FOURCC("SHUT"), GPMF_SAMPLE_RATE_PRECISE, &start, &end);// GPMF_SAMPLE_RATE_FAST);
			data.gpsrate = rate_GPS;
			data.gps_start = start;
			data.gps_end = end;
		}

	cleanup:
		if (payloadres) FreePayloadResource(mp4handle, payloadres);
		if (ms) GPMF_Free(ms);
		CloseSource(mp4handle);
	}

	if (fuzzloopcount == 0 && ret != GPMF_OK)
	{
		if (GPMF_ERROR_UNKNOWN_TYPE == ret)
			printf("Unknown GPMF Type within\n");
		else
			printf("GPMF data has corruption\n");
	}
	else
	{
		ret = GPMF_OK; // when fuzzing, errors reported are showing the system is working.
	}

	return ret;
}


GPMF_ERR GetACCLMP4File(const char* filename, extracted_data &data){
	GPMF_ERR ret = GPMF_OK;
	GPMF_stream metadata_stream = { 0 }, * ms = &metadata_stream;
	double metadatalength;
	uint32_t* payload = NULL;
	uint32_t payloadsize = 0;
	size_t payloadres = 0;
	size_t mp4handle = OpenMP4Source(filename, MOV_GPMF_TRAK_TYPE, MOV_GPMF_TRAK_SUBTYPE, 0);

	if (mp4handle == 0){
		printf("error: %s is an invalid MP4/MOV or it has no GPMF data\n\n", filename);
		return GPMF_ERROR_BAD_STRUCTURE;
	}

	metadatalength = GetDuration(mp4handle);

	if (metadatalength > 0.0){
		uint32_t index, payloads = GetNumberPayloads(mp4handle);

		uint32_t fr_num, fr_dem;
		uint32_t frames = GetVideoFrameRateAndCount(mp4handle, &fr_num, &fr_dem);
		data.nb_frames = frames;
		data.framerate = (double)fr_num/(double)fr_dem;

		for (index = 0; index < payloads; index++){
			double in = 0.0, out = 0.0; //times
			payloadsize = GetPayloadSize(mp4handle, index);
			payloadres = GetPayloadResource(mp4handle, payloadres, payloadsize);
			payload = GetPayload(mp4handle, payloadres, index);
			if (payload == NULL)
				goto cleanup;

			ret = GetPayloadTime(mp4handle, index, &in, &out);
			if (ret != GPMF_OK)
				goto cleanup;

			ret = GPMF_Init(ms, payload, payloadsize);
			if (ret != GPMF_OK)
				goto cleanup;

			while (GPMF_OK == GPMF_FindNext(ms, STR2FOURCC("STRM"), static_cast<GPMF_LEVELS>(GPMF_RECURSE_LEVELS|GPMF_TOLERANT))) //GoPro Hero5/6/7 Accelerometer)
			{
				if (GPMF_OK != GPMF_FindNext(ms, STR2FOURCC("ACCL"), static_cast<GPMF_LEVELS>(GPMF_RECURSE_LEVELS|GPMF_TOLERANT)))
					continue;

				uint32_t samples = GPMF_Repeat(ms);
				uint32_t elements = GPMF_ElementsInStruct(ms);

				uint32_t buffersize = samples * elements * sizeof(double);
				GPMF_stream find_stream;
				double* ptr, * tmpbuffer = (double*)malloc(buffersize);

#define MAX_UNITS	64
#define MAX_UNITLEN	8
				char units[MAX_UNITS][MAX_UNITLEN] = { "" };
				uint32_t unit_samples = 1;

				char complextype[MAX_UNITS] = { "" };
				uint32_t type_samples = 1;

				if (tmpbuffer)
				{
					uint32_t i;
					//Search for any units to display
					GPMF_CopyState(ms, &find_stream);
					if (GPMF_OK == GPMF_FindPrev(&find_stream, GPMF_KEY_SI_UNITS, static_cast<GPMF_LEVELS>(GPMF_CURRENT_LEVEL | GPMF_TOLERANT)) ||
						GPMF_OK == GPMF_FindPrev(&find_stream, GPMF_KEY_UNITS, static_cast<GPMF_LEVELS>(GPMF_CURRENT_LEVEL | GPMF_TOLERANT)))
					{
						char* data = (char*)GPMF_RawData(&find_stream);
						uint32_t ssize = GPMF_StructSize(&find_stream);
						if (ssize > MAX_UNITLEN - 1) ssize = MAX_UNITLEN - 1;
						unit_samples = GPMF_Repeat(&find_stream);

						for (i = 0; i < unit_samples && i < MAX_UNITS; i++)
						{
							memcpy(units[i], data, ssize);
							units[i][ssize] = 0;
							data += ssize;
						}
					}

					if (data.accl_x_unit.empty()) {
						data.accl_x_unit = units[0];
						data.accl_y_unit = units[1];
						data.accl_z_unit = units[2];
					}
					//Search for TYPE if Complex
					GPMF_CopyState(ms, &find_stream);
					type_samples = 0;
					if (GPMF_OK == GPMF_FindPrev(&find_stream, GPMF_KEY_TYPE, static_cast<GPMF_LEVELS>(GPMF_CURRENT_LEVEL | GPMF_TOLERANT)))
					{
						char* data = (char*)GPMF_RawData(&find_stream);
						uint32_t ssize = GPMF_StructSize(&find_stream);
						if (ssize > MAX_UNITLEN - 1) ssize = MAX_UNITLEN - 1;
						type_samples = GPMF_Repeat(&find_stream);

						for (i = 0; i < type_samples && i < MAX_UNITS; i++)
						{
							complextype[i] = data[i];
						}
					}

					if (GPMF_OK == GPMF_ScaledData(ms, tmpbuffer, buffersize, 0, samples, GPMF_TYPE_DOUBLE)){//Output scaled data as floats
						ptr = tmpbuffer;
						int pos = 0;
						for (i = 0; i < samples; i++)	{
							//data.accl_x.emplace_back(*ptr++);
							data.accl_z.emplace_back(*ptr++);
							data.accl_y.emplace_back(*ptr++);
							//data.accl_z.emplace_back(*ptr++);
							data.accl_x.emplace_back(*ptr++);
							/*printf("--%.3f%s, ", data.accl_x.back(), units[0 % unit_samples]);
							printf("--%.3f%s, ", data.accl_y.back(), units[1 % unit_samples]);
							printf("--%.3f%s, ", data.accl_z.back(), units[2 % unit_samples]);
							if (fuzzloopcount == 0) printf("\n");*/
						}
					}
					free(tmpbuffer);
				}
			}
			GPMF_ResetState(ms);
		}

		if (show_computed_samplerates)
		{
			mp4callbacks cbobject;
			cbobject.mp4handle = mp4handle;
			cbobject.cbGetNumberPayloads = GetNumberPayloads;
			cbobject.cbGetPayload = GetPayload;
			cbobject.cbGetPayloadSize = GetPayloadSize;
			cbobject.cbGetPayloadResource = GetPayloadResource;
			cbobject.cbGetPayloadTime = GetPayloadTime;
			cbobject.cbFreePayloadResource = FreePayloadResource;
			cbobject.cbGetEditListOffsetRationalTime = GetEditListOffsetRationalTime;

			double start, end;
			double rate_ACCL = GetGPMFSampleRate(cbobject, STR2FOURCC("ACCL"), STR2FOURCC("SHUT"), GPMF_SAMPLE_RATE_PRECISE, &start, &end);// GPMF_SAMPLE_RATE_FAST);
			data.acclrate = rate_ACCL;
			data.accl_start = start;
			data.accl_end = end;
		}

	cleanup:
		if (payloadres) FreePayloadResource(mp4handle, payloadres);
		if (ms) GPMF_Free(ms);
		CloseSource(mp4handle);
	}

	if (fuzzloopcount == 0 && ret != GPMF_OK)
	{
		if (GPMF_ERROR_UNKNOWN_TYPE == ret)
			printf("Unknown GPMF Type within\n");
		else
			printf("GPMF data has corruption\n");
	}
	else
	{
		ret = GPMF_OK; // when fuzzing, errors reported are showing the system is working.
	}

	return ret;
}

GPMF_ERR GetGYROMP4File(const char* filename, extracted_data &data){
	GPMF_ERR ret = GPMF_OK;
	GPMF_stream metadata_stream = { 0 }, * ms = &metadata_stream;
	double metadatalength;
	uint32_t* payload = NULL;
	uint32_t payloadsize = 0;
	size_t payloadres = 0;
	size_t mp4handle = OpenMP4Source(filename, MOV_GPMF_TRAK_TYPE, MOV_GPMF_TRAK_SUBTYPE, 0);

	if (mp4handle == 0){
		printf("error: %s is an invalid MP4/MOV or it has no GPMF data\n\n", filename);
		return GPMF_ERROR_BAD_STRUCTURE;
	}

	metadatalength = GetDuration(mp4handle);

	if (metadatalength > 0.0){
		uint32_t index, payloads = GetNumberPayloads(mp4handle);

		uint32_t fr_num, fr_dem;
		uint32_t frames = GetVideoFrameRateAndCount(mp4handle, &fr_num, &fr_dem);
		data.nb_frames = frames;
		data.framerate = (double)fr_num/(double)fr_dem;

		for (index = 0; index < payloads; index++){
			double in = 0.0, out = 0.0; //times
			payloadsize = GetPayloadSize(mp4handle, index);
			payloadres = GetPayloadResource(mp4handle, payloadres, payloadsize);
			payload = GetPayload(mp4handle, payloadres, index);
			if (payload == NULL)
				goto cleanup;

			ret = GetPayloadTime(mp4handle, index, &in, &out);
			if (ret != GPMF_OK)
				goto cleanup;

			ret = GPMF_Init(ms, payload, payloadsize);
			if (ret != GPMF_OK)
				goto cleanup;

			while (GPMF_OK == GPMF_FindNext(ms, STR2FOURCC("STRM"), static_cast<GPMF_LEVELS>(GPMF_RECURSE_LEVELS|GPMF_TOLERANT))) //GoPro Hero5/6/7 Accelerometer)
			{

				if (GPMF_OK != GPMF_FindNext(ms, STR2FOURCC("GYRO"), static_cast<GPMF_LEVELS>(GPMF_RECURSE_LEVELS|GPMF_TOLERANT)))
						continue;

				uint32_t samples = GPMF_Repeat(ms);
				uint32_t elements = GPMF_ElementsInStruct(ms);

				uint32_t buffersize = samples * elements * sizeof(double);
				GPMF_stream find_stream;
				double* ptr, * tmpbuffer = (double*)malloc(buffersize);

				#define MAX_UNITS	64
				#define MAX_UNITLEN	8
				char units[MAX_UNITS][MAX_UNITLEN] = { "" };
				uint32_t unit_samples = 1;

				char complextype[MAX_UNITS] = { "" };
				uint32_t type_samples = 1;

				if (tmpbuffer)
				{
					uint32_t i;
					//Search for any units to display
					GPMF_CopyState(ms, &find_stream);
					if (GPMF_OK == GPMF_FindPrev(&find_stream, GPMF_KEY_SI_UNITS, static_cast<GPMF_LEVELS>(GPMF_CURRENT_LEVEL | GPMF_TOLERANT)) ||
						GPMF_OK == GPMF_FindPrev(&find_stream, GPMF_KEY_UNITS, static_cast<GPMF_LEVELS>(GPMF_CURRENT_LEVEL | GPMF_TOLERANT)))
					{
						char* data = (char*)GPMF_RawData(&find_stream);
						uint32_t ssize = GPMF_StructSize(&find_stream);
						if (ssize > MAX_UNITLEN - 1) ssize = MAX_UNITLEN - 1;
						unit_samples = GPMF_Repeat(&find_stream);

						for (i = 0; i < unit_samples && i < MAX_UNITS; i++)
						{
							memcpy(units[i], data, ssize);
							units[i][ssize] = 0;
							data += ssize;
						}
					}

					if (data.gyro_x_unit.empty()) {
						data.gyro_x_unit = units[0];
						data.gyro_y_unit = units[1];
						data.gyro_z_unit = units[2];
					}

					//Search for TYPE if Complex
					GPMF_CopyState(ms, &find_stream);
					type_samples = 0;
					if (GPMF_OK == GPMF_FindPrev(&find_stream, GPMF_KEY_TYPE, static_cast<GPMF_LEVELS>(GPMF_CURRENT_LEVEL | GPMF_TOLERANT)))
					{
						char* data = (char*)GPMF_RawData(&find_stream);
						uint32_t ssize = GPMF_StructSize(&find_stream);
						if (ssize > MAX_UNITLEN - 1) ssize = MAX_UNITLEN - 1;
						type_samples = GPMF_Repeat(&find_stream);

						for (i = 0; i < type_samples && i < MAX_UNITS; i++)
						{
							complextype[i] = data[i];
						}
					}

					if (GPMF_OK == GPMF_ScaledData(ms, tmpbuffer, buffersize, 0, samples, GPMF_TYPE_DOUBLE)){//Output scaled data as floats
						ptr = tmpbuffer;
						int pos = 0;
						for (i = 0; i < samples; i++)	{
							//data.gyro_x.emplace_back(*ptr++);
							data.gyro_z.emplace_back(*ptr++);
							data.gyro_y.emplace_back(*ptr++);
							//data.gyro_z.emplace_back(*ptr++);
							data.gyro_x.emplace_back(*ptr++);
							/*printf("--%.3f%s, ", data.accl_x.back(), units[0 % unit_samples]);
							printf("--%.3f%s, ", data.accl_y.back(), units[1 % unit_samples]);
							printf("--%.3f%s, ", data.accl_z.back(), units[2 % unit_samples]);
							if (fuzzloopcount == 0) printf("\n");*/
						}
					}
					free(tmpbuffer);
				}
			}
			GPMF_ResetState(ms);
		}

		if (show_computed_samplerates)
		{
			mp4callbacks cbobject;
			cbobject.mp4handle = mp4handle;
			cbobject.cbGetNumberPayloads = GetNumberPayloads;
			cbobject.cbGetPayload = GetPayload;
			cbobject.cbGetPayloadSize = GetPayloadSize;
			cbobject.cbGetPayloadResource = GetPayloadResource;
			cbobject.cbGetPayloadTime = GetPayloadTime;
			cbobject.cbFreePayloadResource = FreePayloadResource;
			cbobject.cbGetEditListOffsetRationalTime = GetEditListOffsetRationalTime;

			double start, end;
			double rate_GYRO = GetGPMFSampleRate(cbobject, STR2FOURCC("GYRO"), STR2FOURCC("SHUT"), GPMF_SAMPLE_RATE_PRECISE, &start, &end);// GPMF_SAMPLE_RATE_FAST);
			data.gyrorate = rate_GYRO;
			data.gyro_start = start;
			data.gyro_end = end;
		}

	cleanup:
		if (payloadres) FreePayloadResource(mp4handle, payloadres);
		if (ms) GPMF_Free(ms);
		CloseSource(mp4handle);
	}

	if (fuzzloopcount == 0 && ret != GPMF_OK)
	{
		if (GPMF_ERROR_UNKNOWN_TYPE == ret)
			printf("Unknown GPMF Type within\n");
		else
			printf("GPMF data has corruption\n");
	}
	else
	{
		ret = GPMF_OK; // when fuzzing, errors reported are showing the system is working.
	}

	return ret;
}


//---------------------CLAUDE, fix gps jump------------------------//

// ─────────────────────────────────────────────────────────────────────────────
//  GPS Track Cleaner
//
//  Problem: GoPro (and many other action-cam GPS receivers) occasionally emit
//  samples that are hundreds or thousands of metres away from the real track.
//  They also freeze the position for several samples before and after each
//  glitch, then snap back.  The pattern is always:
//
//      … good … | frozen | JUMP | frozen@wrong_place | JUMP_BACK | frozen | good …
//
//  Two filters are combined to catch all variants without touching good data:
//
//  Filter A – Track proximity (requires known reference points)
//    A sample is suspect if it is farther than `maxDistFromTrackM` from every
//    reference point (finish line + intermediates).  Reference points only need
//    to loosely cover the circuit; a 300–500 m radius easily spans a typical
//    racetrack.
//
//  Filter B – Step distance from the last accepted sample
//    A sample is suspect if it is farther than `maxStepM` from the last sample
//    that was accepted as good.  This catches "return jumps" (the snap-back
//    from the wrong location) that may not be far from the track in absolute
//    terms but are still physically impossible in one timestep.
//
//  A sample is marked BAD only when BOTH filters flag it (logical AND).
//  This avoids false positives from:
//    - GPS quantisation noise (~11 m hops at 4 d.p. precision): step > 0
//      but distance from track is 0 → Filter A passes → sample kept.
//    - Legitimate large steps when re-acquiring signal after a tunnel: step is
//      large but the new position is plausibly on track → Filter A passes.
//
//  Bad segments are replaced by linear interpolation of lat/lon/alt between
//  the last good sample before and the first good sample after the gap.
//  The `interpolated` flag marks which samples were synthesised.
// ─────────────────────────────────────────────────────────────────────────────

namespace detail {

/// Minimum haversine distance from (lat, lon) to any reference point.
inline double minDistToRefs(
    double lat, double lon,
    const std::vector<double>& refLat,
    const std::vector<double>& refLon) noexcept
{
    double minD = std::numeric_limits<double>::max();
    for (std::size_t k = 0; k < refLat.size(); ++k)
        minD = std::min(minD, haversine_meters(lat, lon, refLat[k], refLon[k]));
    return minD;
}

/// Linear interpolation for a scalar.
inline double lerp(double a, double b, double t) noexcept
{
    return a + t * (b - a);
}

} // namespace detail

// ─────────────────────────────────────────────────────────────────────────────
//  Output type
// ─────────────────────────────────────────────────────────────────────────────


// ─────────────────────────────────────────────────────────────────────────────
//  Main cleaning function
// ─────────────────────────────────────────────────────────────────────────────

/// Clean a raw GPS track by removing samples that are implausibly far from the
/// known circuit and linearly interpolating over the removed segments.
///
/// @param lat, lon, alt, ts     Parallel raw GPS vectors (same length, ts
///                              must be monotonically non-decreasing).
/// @param refLat, refLon        Reference points on the circuit (finish line
///                              and/or intermediate checkpoints).  At least
///                              one is required.  More is better — they define
///                              the "valid region" for the track.
/// @param maxDistFromTrackM     Filter A radius.  Samples farther than this
///                              from EVERY reference point are suspect.
///                              Rule of thumb: half the longest straight on the
///                              circuit, or 300–500 m for a typical club track.
/// @param maxStepM              Filter B step limit.  A sample farther than this
///                              from the last *accepted* sample is also suspect
///                              (catches impossible snap-back jumps).
///                              Set to `std::numeric_limits<double>::max()` to
///                              disable Filter B entirely.
///
/// @return  CleanedGPS with the same number of samples as the input.  Bad
///          samples are linearly interpolated; the `interpolated` flag marks
///          which ones were synthesised.


void cleanGPSjumps(extracted_data &data, laps_data &laps, double maxDistFromTrackM, double maxStepM){
        

    // ── Pass 1: build bad[] mask ──────────────────────────────────────────────
    // A sample is BAD if BOTH conditions hold:
    //   A) it is far from all reference points (implausible location)
    //   B) it is far from the last accepted sample (impossible step)
    //
    // Using AND means neither filter alone causes false positives.
    
    //Copy the data to be cleaned. I might not even need that
    vector<double> lat = data.gps_lat;
    vector<double> lon = data.gps_long;
    vector<double> alt = data.gps_alt;
    const std::size_t N = lat.size();
    std::vector<bool> bad(N, false);
    //Get all the track references 
    std::vector<double> refLat, refLon;
    refLat.push_back(laps.finish_lat); refLon.push_back(laps.finish_long);
    for (size_t i = 0; i < laps.intermediate_lat.size(); i++){
        refLat.push_back(laps.intermediate_lat[i]);
        refLon.push_back(laps.intermediate_long[i]);
    }

    std::size_t lastGoodIdx = N; // sentinel: "no good sample seen yet"

    for (std::size_t i = 0; i < N; ++i)
    {
        // ── Filter A: proximity to any known track point ──────────────────
        const double distToTrack = detail::minDistToRefs(lat[i], lon[i], refLat, refLon);
        const bool farFromTrack  = (distToTrack > maxDistFromTrackM);

        // ── Filter B: step distance from last accepted sample ─────────────
        bool impossibleStep = false;
        if (lastGoodIdx < N) // we have a previous good sample
        {
            const double step = haversine_meters(lat[i], lon[i], lat[lastGoodIdx], lon[lastGoodIdx]);
            impossibleStep = (step > maxStepM);
        }

        // ── Combined decision ─────────────────────────────────────────────
        if (farFromTrack && (lastGoodIdx == N || impossibleStep))
        {
            // Both filters agree: this sample is a GPS glitch
            bad[i] = true;
        }
        else
        {
            lastGoodIdx = i;
        }
    }

    // ── Pass 2: interpolate over bad segments ─────────────────────────────────
    // For each contiguous run of bad samples, linearly interpolate between the
    // last good sample before the run and the first good sample after it.
    // If the run touches the start or end of the file, we can only copy from
    // the single valid neighbour (or leave the edge flagged).

    
    vector<bool> interpolated = std::vector<bool>(N, false);
    int nBadRemoved  = 0, nGapsRemaining = 0;

    std::size_t i = 0;
    while (i < N)
    {
        if (!bad[i]) { ++i; continue; }

        // Found the start of a bad run.  Find its end.
        std::size_t runStart = i;
        while (i < N && bad[i]) ++i;
        std::size_t runEnd = i; // first good sample after the run (exclusive)

        nBadRemoved += (runEnd - runStart);

        // Find bounding good samples
        // prevGood: last good sample index before the run
        // nextGood: first good sample index after the run
        const bool hasPrev = (runStart > 0);                // run doesn't touch file start
        const bool hasNext = (runEnd   < N);                // run doesn't touch file end

        if (hasPrev && hasNext)
        {
            // Standard case: interpolate between prevGood and nextGood
            const std::size_t p = runStart - 1;
            const std::size_t n = runEnd;
            const double totalDt = data.gps_ts[n] - data.gps_ts[p];

            for (std::size_t j = runStart; j < runEnd; ++j)
            {
                const double t = (totalDt > 0.0)
                               ? (data.gps_ts[j] - data.gps_ts[p]) / totalDt
                               : static_cast<double>(j - p) / static_cast<double>(n - p);

                data.gps_lat[j]    = detail::lerp(data.gps_lat[p], data.gps_lat[n], t);
                data.gps_long[j]   = detail::lerp(data.gps_long[p], data.gps_long[n], t);
                data.gps_alt[j]    = detail::lerp(data.gps_alt[p], data.gps_alt[n], t);
                interpolated[j] = true;
            }

        }
        else if (hasPrev && !hasNext)
        {
            // Run touches the end: copy last good value forward
            const std::size_t p = runStart - 1;
            for (std::size_t j = runStart; j < runEnd; ++j)
            {
                data.gps_lat[j]   = data.gps_lat[p];
                data.gps_long[j]  = data.gps_long[p];
                data.gps_alt[j]   = data.gps_alt[p];
                interpolated[j] = true;
                ++nGapsRemaining;
            }
        }
        else if (!hasPrev && hasNext)
        {
            // Run touches the start: copy first good value backward
            const std::size_t n = runEnd;
            for (std::size_t j = runStart; j < runEnd; ++j)
            {
                data.gps_lat[j]   = data.gps_lat[n];
                data.gps_long[j]  = data.gps_long[n];
                data.gps_alt[j]   = data.gps_alt[n];
                interpolated[j] = true;
                ++nGapsRemaining;
            }
        }
        else
        {
            // Entire file is bad — nothing we can do.
            ++nGapsRemaining;
        }
    }

    cerr << "Number of corrected data point: " << nBadRemoved << "/" << N << endl;

}