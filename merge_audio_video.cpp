//
// Created by yohan on 31.07.24.
//

#include "merge_audio_video.h"

void concatenate_audio_video(const char* video_file, const char* audio_file, const char* output_file) {
    AVFormatContext* video_fmt_ctx = nullptr;
    AVFormatContext* audio_fmt_ctx = nullptr;
    AVFormatContext* output_fmt_ctx = nullptr;
    AVOutputFormat* output_fmt = nullptr;
    AVStream* video_stream = nullptr;
    AVStream* audio_stream = nullptr;
    AVPacket* pkt = nullptr;
    int video_index = -1, audio_index = -1;

    // Open input video file
    if (avformat_open_input(&video_fmt_ctx, video_file, nullptr, nullptr) < 0) {
        std::cerr << "Could not open input video file." << std::endl;
        exit(1);
    }
    if (avformat_find_stream_info(video_fmt_ctx, nullptr) < 0) {
        std::cerr << "Could not find stream information in video file." << std::endl;
        exit(1);
    }

    // Open input audio file
    if (avformat_open_input(&audio_fmt_ctx, audio_file, nullptr, nullptr) < 0) {
        std::cerr << "Could not open input audio file." << std::endl;
        exit(1);
    }
    if (avformat_find_stream_info(audio_fmt_ctx, nullptr) < 0) {
        std::cerr << "Could not find stream information in audio file." << std::endl;
        exit(1);
    }

    // Create output context
    avformat_alloc_output_context2(&output_fmt_ctx, nullptr, nullptr, output_file);
    if (!output_fmt_ctx) {
        std::cerr << "Could not create output context." << std::endl;
        exit(1);
    }
    //output_fmt = output_fmt_ctx->oformat;

    // Find video and audio streams
    for (unsigned i = 0; i < video_fmt_ctx->nb_streams; i++) {
        if (video_fmt_ctx->streams[i]->codecpar->codec_type == AVMEDIA_TYPE_VIDEO) {
            video_stream = avformat_new_stream(output_fmt_ctx, nullptr);
            if (!video_stream) {
                std::cerr << "Failed allocating output video stream." << std::endl;
                exit(1);
            }
            avcodec_parameters_copy(video_stream->codecpar, video_fmt_ctx->streams[i]->codecpar);
            video_stream->codecpar->codec_tag = 0;
            video_index = i;
            break;
        }
    }

    for (unsigned i = 0; i < audio_fmt_ctx->nb_streams; i++) {
        if (audio_fmt_ctx->streams[i]->codecpar->codec_type == AVMEDIA_TYPE_AUDIO) {
            audio_stream = avformat_new_stream(output_fmt_ctx, nullptr);
            if (!audio_stream) {
                std::cerr << "Failed allocating output audio stream." << std::endl;
                exit(1);
            }
            avcodec_parameters_copy(audio_stream->codecpar, audio_fmt_ctx->streams[i]->codecpar);
            audio_stream->codecpar->codec_tag = 0;
            audio_index = i;
            break;
        }
    }

    // Open output file
    //if (!(output_fmt->flags & AVFMT_NOFILE)) {
    if (!(output_fmt_ctx->oformat->flags & AVFMT_NOFILE)) {
        if (avio_open(&output_fmt_ctx->pb, output_file, AVIO_FLAG_WRITE) < 0) {
            std::cerr << "Could not open output file." << std::endl;
            exit(1);
        }
    }

    // Write output file header
    if (avformat_write_header(output_fmt_ctx, nullptr) < 0) {
        std::cerr << "Error occurred when writing header to output file." << std::endl;
        exit(1);
    }

    pkt = av_packet_alloc();

    // Read video packets and write them to the output file
    while (av_read_frame(video_fmt_ctx, pkt) >= 0) {
        if (pkt->stream_index == video_index) {
            pkt->stream_index = video_stream->index;
            av_packet_rescale_ts(pkt, video_fmt_ctx->streams[video_index]->time_base, video_stream->time_base);
            if (av_interleaved_write_frame(output_fmt_ctx, pkt) < 0) {
                std::cerr << "Error writing video frame to output file." << std::endl;
                break;
            }
        }
        av_packet_unref(pkt);
    }

    // Read audio packets and write them to the output file
    while (av_read_frame(audio_fmt_ctx, pkt) >= 0) {
        if (pkt->stream_index == audio_index) {
            pkt->stream_index = audio_stream->index;
            av_packet_rescale_ts(pkt, audio_fmt_ctx->streams[audio_index]->time_base, audio_stream->time_base);
            if (av_interleaved_write_frame(output_fmt_ctx, pkt) < 0) {
                std::cerr << "Error writing audio frame to output file." << std::endl;
                break;
            }
        }
        av_packet_unref(pkt);
    }

    // Write output file trailer
    av_write_trailer(output_fmt_ctx);

    // Cleanup
    av_packet_free(&pkt);
    avformat_close_input(&video_fmt_ctx);
    avformat_close_input(&audio_fmt_ctx);
    //if (!(output_fmt->flags & AVFMT_NOFILE)) {
    if (!(output_fmt_ctx->oformat->flags & AVFMT_NOFILE)) {
        avio_closep(&output_fmt_ctx->pb);
    }
    avformat_free_context(output_fmt_ctx);

    std::cout << "Audio and video concatenation completed." << std::endl;
}

