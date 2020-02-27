#!/usr/bin/env python

from subprocess import call
from FRC_utils.image_proccessing_utils import get_camera_by_id
from FRC_utils.ros_utils import get_config
from unicodedata import normalize


def start_stream(cam_id):
    # res = 'stream_quality 10 \nstream_maxrate 100 \nstream_port 5810 \nstream_localhost off \noutput_pictures off ' \
    #       '\nframerate 25 \nffmpeg_video_codec mpeg4 \nwidth 720 ' \
    #       '\nheight 640 \nsaturation 0 \nauto_brightness off \nvideodevice /dev/video' + str(cam_id)

    # call('chmod 777 /home/bob/.motion/ -R', shell=True)
    # with open('/home/bob/.motion/motion.conf', 'r') as f:
    #     f.write(res)
    call('motion', shell=True)


def main():
    start_stream(0)


if __name__ == '__main__':
    main()
