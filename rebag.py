#!/usr/bin/env python

from io import BytesIO
import os
from PIL import Image
import rosbag
from sensor_msgs.msg import CompressedImage


bagfile_name = 'head.bag'
elements = os.path.splitext(bagfile_name)
output_bagfile_name = elements[0] + "_compressed." + elements[1]


def compress_msg(raw_msg):
    img = Image.frombytes("L", (msg.width, msg.height), msg.data,
                          'raw', "L", 0, 1)
    # img.show()
    output = BytesIO()
    img.save(output, format='png')
    output.flush()

    # Show the image for debug
    # output.seek(0)
    # i = Image.open(output)
    # i.show()
    # break

    output_msg = CompressedImage()
    output_msg.header = msg.header
    output_msg.format = 'png'
    output_msg.data = output.getvalue()
    return output_msg

with rosbag.Bag(bagfile_name) as bag:
    with rosbag.Bag(output_bagfile_name, 'w') as outbag:
        for topic, msg, t in bag.read_messages():

            print("topic is %s, time is %s" % (topic, t))
            if topic.endswith('image_raw'):
                try:
                    msg = compress_msg(msg)
                except Exception as ex:
                    print("Exception: %s when parsing msg." % ex)
                topic = topic[:-9] + "compressed"
            # print("%s" % output_msg)
            outbag.write(topic, msg, t)
