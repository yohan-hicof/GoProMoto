//
// Created by yohan on 31.07.24.
//

#ifndef GOPROMOTO_MERGE_AUDIO_VIDEO_H
#define GOPROMOTO_MERGE_AUDIO_VIDEO_H

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

void concatenate_audio_video(const char* video_file, const char* audio_file, const char* output_file);

#endif //GOPROMOTO_MERGE_AUDIO_VIDEO_H
