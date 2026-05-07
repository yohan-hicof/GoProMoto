//
// Created by yohan on 30.07.24.
//


#include "merge_audio.h"

#if 1
int concatenate_audio_streams(const vector<string> &list_input, const string &output_filename, double start_ts, double end_ts) {
    AVFormatContext* audio_fmt_ctx1 = nullptr;
    AVFormatContext* audio_fmt_ctx2 = nullptr;
    AVFormatContext* output_fmt_ctx = nullptr;
    AVStream* out_stream = nullptr;
    AVPacket* pkt = nullptr;
    int audio_stream_index1 = -1, audio_stream_index2 = -1;
    int64_t frames_pts = 0, last_frames_pts = 0;

    if (list_input.empty()) return -1;

    // Open first input audio file
    if (avformat_open_input(&audio_fmt_ctx1, list_input[0].c_str(), nullptr, nullptr) < 0) {
        std::cerr << "Could not open input audio file 1." << std::endl;
        return -1;
    }
    if (avformat_find_stream_info(audio_fmt_ctx1, nullptr) < 0) {
        std::cerr << "Could not find stream information in audio file 1." << std::endl;
        return -1;
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
        return -1;
    }

    // Create an audio stream in the output file using codec parameters from the first input stream
    out_stream = avformat_new_stream(output_fmt_ctx, nullptr);
    if (!out_stream) {
        std::cerr << "Failed to allocate output stream." << std::endl;
        return -1;
    }

    // Copy codec parameters from the first input file's audio stream
    if (avcodec_parameters_copy(out_stream->codecpar, audio_fmt_ctx1->streams[audio_stream_index1]->codecpar) < 0) {
        std::cerr << "Failed to copy codec parameters to output stream." << std::endl;
        return -1;
    }

    // Set codec tag to 0, as it will be set automatically
    out_stream->codecpar->codec_tag = 0;

    // Open output file
    if (!(output_fmt_ctx->oformat->flags & AVFMT_NOFILE)) {
        if (avio_open(&output_fmt_ctx->pb, output_filename.c_str(), AVIO_FLAG_WRITE) < 0) {
            std::cerr << "Could not open output file." << std::endl;
            return -1;
        }
    }

    // Write output file header
    if (avformat_write_header(output_fmt_ctx, nullptr) < 0) {
        std::cerr << "Error occurred when writing header to output file." << std::endl;
        return -1;
    }

    pkt = av_packet_alloc();

    ///------------------CLAUDE, to skip audio and sync with video if cropped----------------///
    const AVStream* audio_in1  = audio_fmt_ctx1->streams[audio_stream_index1];
    const int64_t   start_pts = av_rescale_q( (int64_t)(start_ts * AV_TIME_BASE), AV_TIME_BASE_Q, audio_in1->time_base);   // scale to audio ticks
    const int64_t   stop_pts = av_rescale_q( (int64_t)(end_ts * AV_TIME_BASE), AV_TIME_BASE_Q, audio_in1->time_base) - start_pts;

    if (start_ts > 0.0)
        av_seek_frame(audio_fmt_ctx1, audio_stream_index1,  start_pts, AVSEEK_FLAG_BACKWARD);

    int64_t pts_offset = AV_NOPTS_VALUE;


    // Copy packets from the first input file to the output file
    while (av_read_frame(audio_fmt_ctx1, pkt) >= 0) {

        if (pkt->stream_index == audio_stream_index1) {
            if (pkt->pts < start_pts) {
                av_packet_unref(pkt);
                continue;
            }
            if (pkt->pts > stop_pts){
                av_packet_unref(pkt);
                break;
            }

            if (pts_offset == AV_NOPTS_VALUE)
                pts_offset = pkt->pts;

            pkt->pts -= pts_offset;
            pkt->dts  = pkt->pts;
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
                if (pkt->pts > stop_pts){
                    av_packet_unref(pkt);
                    break;
                }
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
    return 0;
}

#else

int concatenate_audio_streams(const vector<string>& list_input,  const string& output_filename, double start_ts, double end_ts){

    AVFormatContext* in_fmt_ctx  = nullptr;
    AVFormatContext* out_fmt_ctx = nullptr;
    AVStream*        out_stream  = nullptr;
    AVPacket*        pkt         = nullptr;

    // Tracks the next PTS to use in the output, in out_stream->time_base.
    // Updated after every written packet so files chain seamlessly.
    int64_t out_pts_offset = 0;

    // PTS of the first kept packet in file 0, used to normalise output to 0.
    int64_t pts_first = AV_NOPTS_VALUE;

    for (size_t i = 0; i < list_input.size(); i++) {

        if (avformat_open_input(&in_fmt_ctx, list_input[i].c_str(), nullptr, nullptr) < 0) {
            cerr << "Could not open " << list_input[i] << endl;
            return -1;
        }
        if (avformat_find_stream_info(in_fmt_ctx, nullptr) < 0) {
            cerr << "Could not find stream info in " << list_input[i] << endl;
            return -1;
        }

        int audio_idx = -1;
        for (unsigned j = 0; j < in_fmt_ctx->nb_streams; j++) {
            if (in_fmt_ctx->streams[j]->codecpar->codec_type == AVMEDIA_TYPE_AUDIO) {
                audio_idx = j;
                break;
            }
        }
        if (audio_idx < 0) {
            cerr << "No audio stream found in " << list_input[i] << ", skipping." << endl;
            avformat_close_input(&in_fmt_ctx);
            continue;
        }

        AVStream* in_stream = in_fmt_ctx->streams[audio_idx];

        // ── First valid file: set up the output context ───────────────────────
        if (!out_fmt_ctx) {
            avformat_alloc_output_context2(&out_fmt_ctx, nullptr, nullptr, output_filename.c_str());
            if (!out_fmt_ctx) {
                cerr << "Could not create output context." << endl; return -1;
            }
            out_stream = avformat_new_stream(out_fmt_ctx, nullptr);
            if (!out_stream) {
                cerr << "Could not allocate output stream." << endl; return -1;
            }
            if (avcodec_parameters_copy(out_stream->codecpar, in_stream->codecpar) < 0) {
                cerr << "Could not copy codec parameters." << endl; return -1;
            }
            out_stream->codecpar->codec_tag = 0;

            if (!(out_fmt_ctx->oformat->flags & AVFMT_NOFILE)) {
                if (avio_open(&out_fmt_ctx->pb, output_filename.c_str(), AVIO_FLAG_WRITE) < 0) {
                    cerr << "Could not open output file." << endl; return -1;
                }
            }
            if (avformat_write_header(out_fmt_ctx, nullptr) < 0) {
                cerr << "Could not write output header." << endl; return -1;
            }
            pkt = av_packet_alloc();
        }

        // ── target_pts: where to start reading inside this file ───────────────
        // Only relevant for file 0; all other files are read from their beginning.
        int64_t target_pts = 0;
        if (i == 0 && start_ts > 0.0) {
            target_pts = av_rescale_q((int64_t)(start_ts * AV_TIME_BASE),
                                       AV_TIME_BASE_Q, in_stream->time_base);
            av_seek_frame(in_fmt_ctx, audio_idx, target_pts, AVSEEK_FLAG_BACKWARD);
        }

        // ── stop_pts: where to stop reading inside this file ──────────────────
        // end_ts and start_ts are in the original video timeline (file 0's clock).
        // For file 0:  stop_pts is simply end_ts expressed in the file's time base.
        // For file i>0: the file's content starts at (start_ts + seconds_written)
        //               in the original timeline, so the stop point within this
        //               file = end_ts - start_ts - seconds_already_written.
        int64_t stop_pts = AV_NOPTS_VALUE;
        if (end_ts > 0.0) {
            double stop_time_in_file;
            if (i == 0) {
                stop_time_in_file = end_ts;
            } else {
                double seconds_written = out_pts_offset * av_q2d(out_stream->time_base);
                stop_time_in_file = end_ts - start_ts - seconds_written;
            }

            if (stop_time_in_file <= 0.0) {
                // end_ts was already reached in a previous file
                avformat_close_input(&in_fmt_ctx);
                break;
            }
            stop_pts = av_rescale_q((int64_t)(stop_time_in_file * AV_TIME_BASE),
                                     AV_TIME_BASE_Q, in_stream->time_base);
        }

        // ── Read and write packets ────────────────────────────────────────────
        bool done = false;
        while (av_read_frame(in_fmt_ctx, pkt) >= 0) {
            if (pkt->stream_index == audio_idx) {

                // Skip packets before start_ts (file 0 only — seek is imprecise)
                if (pkt->pts < target_pts) {
                    av_packet_unref(pkt);
                    continue;
                }

                // Stop when we reach end_ts
                if (stop_pts != AV_NOPTS_VALUE && pkt->pts >= stop_pts) {
                    av_packet_unref(pkt);
                    done = true;
                    break;
                }

                // ── PTS normalisation ────────────────────────────────────────
                // For file 0: subtract the first kept packet's pts so output
                //             starts at 0, then rescale to output time base.
                // For file i>0: rescale first (handles different time bases
                //               across clips), then add the cumulative offset.
                if (i == 0) {
                    if (pts_first == AV_NOPTS_VALUE)
                        pts_first = pkt->pts;
                    pkt->pts -= pts_first;
                    pkt->dts  = pkt->pts;
                    av_packet_rescale_ts(pkt, in_stream->time_base, out_stream->time_base);
                } else {
                    av_packet_rescale_ts(pkt, in_stream->time_base, out_stream->time_base);
                    pkt->pts += out_pts_offset;
                    pkt->dts  = pkt->pts;
                }

                pkt->stream_index = out_stream->index;
                out_pts_offset    = pkt->pts + 1; // carry forward for next file

                if (av_interleaved_write_frame(out_fmt_ctx, pkt) < 0) {
                    cerr << "Error writing packet from " << list_input[i] << endl;
                    done = true;
                    break;
                }
            }
            av_packet_unref(pkt);
        }

        avformat_close_input(&in_fmt_ctx);
        if (done) break;
    }

    // ── Finalise output ───────────────────────────────────────────────────────
    if (out_fmt_ctx) {
        av_write_trailer(out_fmt_ctx);
        av_packet_free(&pkt);
        if (!(out_fmt_ctx->oformat->flags & AVFMT_NOFILE))
            avio_closep(&out_fmt_ctx->pb);
        avformat_free_context(out_fmt_ctx);
    }

    cout << "Audio concatenation completed." << endl;
    return 0;
}

#endif