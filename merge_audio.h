//
// Created by yohan on 30.07.24.
//

#ifndef GOPROMOTO_MERGE_AUDIO_H
#define GOPROMOTO_MERGE_AUDIO_H

extern "C" {
    #include <libavformat/avformat.h>
    #include <libavcodec/avcodec.h>
    #include <libavutil/avutil.h>
    #include <libavutil/opt.h>
    #include <libswresample/swresample.h>

}
#include <iostream>
#include <vector>

using namespace std;

int concatenate_audio_streams(const vector<string> &list_input, const string &output_filename, double start_ts=0, double end_ts=-1);
#endif //GOPROMOTO_MERGE_AUDIO_H
