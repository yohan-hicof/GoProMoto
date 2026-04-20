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

	do{
		ret = GetGPSMP4File(filename, data);
	} while (ret == GPMF_OK && --fuzzloopcount > 0);

    correct_gps_data(data);

	do{
		ret = GetACCLMP4File(filename, data);
	} while (ret == GPMF_OK && --fuzzloopcount > 0);

	do{
		ret = GetGYROMP4File(filename, data);
	} while (ret == GPMF_OK && --fuzzloopcount > 0);

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

    if (nb_data_points == 0) nb_data_points = data.gps_lat.size();

    ofstream file(filename);

    file << "Framerate: " << data.framerate << "\n";
    file << "Nb Frames: " << data.nb_frames << "\n";

    file << "---GPS---\n";
    file << "GPS start: " << data.gps_start << "\n";
    file << "GPS end: " << data.gps_end << "\n";
    file << "GPS lat, long, alt, speed, speed2, accl_x, accl_y, accl_z, gyro_x, gyro_y, gyro_z" << "\n";
    for (size_t i = 0; i < nb_data_points; i++){
        file << data.gps_lat[i] <<","<< data.gps_long[i] <<","<< data.gps_alt[i] <<",";
        file << data.gps_speed[i] <<","<< data.gps_speed2[i];
        file << "," << data.accl_x[i] << "," << data.accl_y[i] << "," << data.accl_z[i];
        file << "," << data.gyro_x[i] << "," << data.gyro_y[i] << "," << data.gyro_z[i] <<"\n";
    }
    //TODO The same with the other data when I need to debug them.
    file.close();

}

GPMF_ERR GetGPSMP4File(const char* filename, extracted_data &data){
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
