//
// Created by yohan on 30.07.24.
//


#include "merge_audio.h"


void concatenate_audio_streams(const vector<string> &list_input, const string &output_filename) {
    AVFormatContext* audio_fmt_ctx1 = nullptr;
    AVFormatContext* audio_fmt_ctx2 = nullptr;
    AVFormatContext* output_fmt_ctx = nullptr;
    AVStream* out_stream = nullptr;
    AVPacket* pkt = nullptr;
    int audio_stream_index1 = -1, audio_stream_index2 = -1;
    int64_t frames_pts = 0, last_frames_pts = 0;

    if (list_input.empty()) return;

    // Open first input audio file
    if (avformat_open_input(&audio_fmt_ctx1, list_input[0].c_str(), nullptr, nullptr) < 0) {
        std::cerr << "Could not open input audio file 1." << std::endl;
        exit(1);
    }
    if (avformat_find_stream_info(audio_fmt_ctx1, nullptr) < 0) {
        std::cerr << "Could not find stream information in audio file 1." << std::endl;
        exit(1);
    }

    // Find audio stream in input files
    for (unsigned i = 0; i < audio_fmt_ctx1->nb_streams; i++) {
        if (audio_fmt_ctx1->streams[i]->codecpar->codec_type == AVMEDIA_TYPE_AUDIO) {
            audio_stream_index1 = i;
            break;
        }
    }

    // Create output context
    avformat_alloc_output_context2(&output_fmt_ctx, nullptr, nullptr, output_filename.c_str());
    if (!output_fmt_ctx) {
        std::cerr << "Could not create output context." << std::endl;
        exit(1);
    }

    // Create an audio stream in the output file using codec parameters from the first input stream
    out_stream = avformat_new_stream(output_fmt_ctx, nullptr);
    if (!out_stream) {
        std::cerr << "Failed to allocate output stream." << std::endl;
        exit(1);
    }

    // Copy codec parameters from the first input file's audio stream
    if (avcodec_parameters_copy(out_stream->codecpar, audio_fmt_ctx1->streams[audio_stream_index1]->codecpar) < 0) {
        std::cerr << "Failed to copy codec parameters to output stream." << std::endl;
        exit(1);
    }

    // Set codec tag to 0, as it will be set automatically
    out_stream->codecpar->codec_tag = 0;

    // Open output file
    if (!(output_fmt_ctx->oformat->flags & AVFMT_NOFILE)) {
        if (avio_open(&output_fmt_ctx->pb, output_filename.c_str(), AVIO_FLAG_WRITE) < 0) {
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

    // Copy packets from the first input file to the output file
    while (av_read_frame(audio_fmt_ctx1, pkt) >= 0) {
        if (pkt->stream_index == audio_stream_index1) {
            pkt->stream_index = out_stream->index;
            frames_pts = pkt->pts+1;
            av_packet_rescale_ts(pkt, audio_fmt_ctx1->streams[audio_stream_index1]->time_base, out_stream->time_base);
            if (av_interleaved_write_frame(output_fmt_ctx, pkt) < 0) {
                std::cerr << "Error writing audio frame to output file." << std::endl;
                break;
            }
        }
        av_packet_unref(pkt);
    }

    //Now do the following input files
    for (size_t i = 1; i < list_input.size(); i++){
        if (avformat_open_input(&audio_fmt_ctx2, list_input[i].c_str(), nullptr, nullptr) < 0) {
            std::cerr << "Could not open input audio file 2." << std::endl;
            exit(1);
        }
        if (avformat_find_stream_info(audio_fmt_ctx2, nullptr) < 0) {
            std::cerr << "Could not find stream information in audio file 2." << std::endl;
            exit(1);
        }
        for (unsigned j = 0; j < audio_fmt_ctx2->nb_streams; j++) {
            if (audio_fmt_ctx2->streams[j]->codecpar->codec_type == AVMEDIA_TYPE_AUDIO) {
                audio_stream_index2 = j;
                break;
            }
        }
        // Copy packets from the second input file to the output file
        while (av_read_frame(audio_fmt_ctx2, pkt) >= 0) {
            if (pkt->stream_index == audio_stream_index2) {
                pkt->stream_index = out_stream->index;
                pkt->pts += frames_pts; // Increment pts to make it monotonic
                pkt->dts = pkt->pts;
                last_frames_pts = pkt->pts;
                av_packet_rescale_ts(pkt, audio_fmt_ctx2->streams[audio_stream_index2]->time_base, out_stream->time_base);
                if (av_interleaved_write_frame(output_fmt_ctx, pkt) < 0) {
                    std::cerr << "Error writing audio frame to output file." << std::endl;
                    break;
                }
            }
            av_packet_unref(pkt);
        }
        frames_pts = last_frames_pts; //Needed to ensure coherent frame numbers.
    }

    // Write output file trailer
    av_write_trailer(output_fmt_ctx);

    // Cleanup
    av_packet_free(&pkt);
    avformat_close_input(&audio_fmt_ctx1);
    avformat_close_input(&audio_fmt_ctx2);
    if (!(output_fmt_ctx->oformat->flags & AVFMT_NOFILE)) {
        avio_closep(&output_fmt_ctx->pb);
    }
    avformat_free_context(output_fmt_ctx);

    std::cout << "Audio concatenation completed" << std::endl;
}