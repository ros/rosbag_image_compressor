#!/usr/bin/env python

# Copyright 2014 Open Source Robotics Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


from io import BytesIO
import os
from PIL import Image
import rosbag
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image as smImage
from std_msgs.msg import String


def compress_image(msg):
    """
    Take a sensor_msgs/Image
    return a sensor_msgs/CompressedImage
    """
    # fromstring is not available on precise python-imaging v 1.1.7
    # but has been deprecated in v 2.0 using it for now while we need
    # to support precise
    # img = Image.frombytes("L", (msg.width, msg.height), msg.data,
    #                       'raw', "L", 0, 1)
    img = Image.fromstring("L", (msg.width, msg.height), msg.data,
                           'raw', "L", 0, 1)
    # img.show()
    output = BytesIO()
    img.save(output, format='png')
    output.flush()

    output_msg = CompressedImage()
    output_msg.header = msg.header
    output_msg.format = 'png'
    output_msg.data = output.getvalue()

    encoding_msg = String()
    encoding_msg.data = msg.encoding
    return output_msg, encoding_msg


def uncompress_image(compressed_msg, encoding):
    """
    Take a sensor_msgs/CompressedImage and encoding
    This will assume the compression has ignored the encoding and
    will apply the encoding
    return a sensor_msgs/Image
    """
    fh = BytesIO(compressed_msg.data)
    img = Image.open(fh)

    output_msg = smImage()
    output_msg.header = compressed_msg.header
    output_msg.width, output_msg.height = img.size
    output_msg.encoding = encoding
    output_msg.is_bigendian = False  # TODO
    output_msg.step = output_msg.width
    output_msg.data = img.tostring()
    return output_msg


def image_topic_basename(topic):
    """ A convenience method for stripping the endings off an image topic"""
    endings = ['compressed', 'encoding', 'image_raw']
    for e in endings:
        if topic.endswith(e):
            return topic[:-1 * len(e)]
    raise Exception("invalid topic for truncation %s looking for endings %s" %
                    (topic, endings))


class EncodingCache:

    """ A class for caching the encoding type for each topic. This will only
    work if the encoding does not change. """

    def __init__(self):
        self.encoding_map = {}

    def lookup_encoding(self, topic):
        if topic in self.encoding_map:
            return self.encoding_map[topic]
        else:
            raise Exception("failed to find encoding for topic %s in %s" %
                            (topic, self.encoding_map))

    def insert_encoding(self, topic, encoding):
        self.encoding_map[topic] = encoding


def compress(bagfile_in, bagfile_out):
    """ Iterate over bagfile_in and compress images into bagfile_out """
    with rosbag.Bag(bagfile_in) as bag:
        with rosbag.Bag(bagfile_out, 'w') as outbag:
            process_log = {}
            print("Compressing %s into %s" % (bagfile_in, bagfile_out))
            for topic, msg, t in bag.read_messages():
                if topic.endswith('image_raw'):
                    # print("compressing %s, time is %s" % (topic, t))
                    bname = image_topic_basename(topic)
                    try:
                        msg, encoding_msg = compress_image(msg)
                        encoding_topic = bname + "encoding"
                        topic = bname + "compressed"
                        outbag.write(encoding_topic, encoding_msg, t)
                        if bname in process_log:
                            process_log[bname] += 1
                        else:
                            process_log[bname] = 1
                    except Exception as ex:
                        print("Exception: %s when parsing msg. Not compressing" % ex)

                # print("%s" % output_msg)
                outbag.write(topic, msg, t)
            if not process_log:
                print("No images compressed")
            for t, c in process_log.items():
                print("Compressed %s message on topic %simage_raw" % (c, t))


def uncompress(bagfile_in, bagfile_out):
    """ Iterate over bagfile_in and decompress images into bagfile_out """
    with rosbag.Bag(bagfile_in) as bag:
        with rosbag.Bag(bagfile_out, 'w') as outbag:
            process_log = {}
            print("Decompressing %s into %s" % (bagfile_in, bagfile_out))
            encoding_cache = EncodingCache()
            for topic, msg, t in bag.read_messages():
                bname = image_topic_basename(topic)
                if topic.endswith('encoding'):
                    encoding_cache.insert_encoding(bname, msg.data)
                    continue  # do not rewrite the encoding message
                if topic.endswith('compressed'):
                    # print("uncompressing %s, time is %s" % (topic, t))
                    try:
                        enc = encoding_cache.lookup_encoding(bname)
                        msg = uncompress_image(msg, enc)
                        topic = bname + 'image_raw'
                        if bname in process_log:
                            process_log[bname] += 1
                        else:
                            process_log[bname] = 1
                    except Exception as ex:
                        print("Exception: %s when parsing msg. Not decompressing" % ex)

                # print("%s" % output_msg)
                outbag.write(topic, msg, t)
            if not process_log:
                print("No images decompressed")
            for t, c in process_log.items():
                print("Decompressed %s message on topic %scompressed" % (c, t))
