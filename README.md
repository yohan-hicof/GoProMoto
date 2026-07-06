This software takes gopro video input, and generate the overlay for video of motorcycle on racetrack.
It does the following overlays:
 - Current speed bottom left.
 - Current lean angle bottom left, right of the speed.
 - Lap time, current, previous best. Bottom right.
 - Tracks display, current position, track name, video time, lean angle. Top right.

This software has a few dependencies.
 - gpmf parser: https://github.com/gopro/gpmf-parser, it is included here as I modified it a bit.
 - Opencv, the main library to read/write the video.
 - ffmpeg, to read the video in the background for opencv.
 - cairo, to draw over the opencv video frames. Could be done with opencv, but cairo is much more powerfull for that.

The software is under MIT license.

The provided cmake should allow to compile under any OS, but I only tested it with Ubuntu.

The options are the following
 - -i /path/to/video.mp4: The path to the first video. If the video is with the GoPro format, the software will find the subsequent files. If several -i, will just process them in given order, producing a single output.
 - -o /path/to/output.mp4: The path to the output video.
 - -path (-p) /path/to/folder: Will process all the videos in the given folder. For GoPro, find the subsequent videos. Generate the output according to the track name and date.
 - -t /path/to/tracks.csv: A file containing the tracks informations: Name, starting line position (GPS), intermediate positions.
 - -auto_crop (-ac): Will automatically crop around the full lap in the video. If no lap found, will leave the video as is.
 - -best_lap (-bl): Will only display the best lap of the video. The time of previous lap is still displayed.
 - -time_only (-to): Do not process the video and only display the computed lap time.


TODO
 - Add hardcoded tracks.
 - Find a way to have different track configuration (ADR, Ales)
 - Add acceleration/braking UID
 - Change the track to a json files. Easier to maintain and add functionalities.
 - Add a proper way to deal with options.


