#!/usr/bin/env python

import sys

from io import BytesIO
import os
from PIL import Image
import rosbag
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image as smImage
from std_msgs.msg import String

bagfile_name = 'head.bag'
elements = os.path.splitext(bagfile_name)
output_bagfile_name = elements[0] + "_compressed" + elements[1]
uncompressed_bagfile_name = elements[0] + "_uncompressed" + elements[1]


def compress_image(msg):
    img = Image.frombytes("L", (msg.width, msg.height), msg.data,
                          'raw', "L", 0, 1)
    # img.show()
    output = BytesIO()
    img.save(output, format='png')
    output.flush()

    if False:
        # Show the image for debug
        output.seek(0)
        i = Image.open(output)
        i.show()
        sys.exit(0)

    output_msg = CompressedImage()
    output_msg.header = msg.header
    output_msg.format = 'png'
    output_msg.data = output.getvalue()

    encoding_msg = String()
    encoding_msg.data = msg.encoding
    return output_msg, encoding_msg


def uncompress_image(compressed_msg, encoding):
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
    endings = ['compressed', 'encoding', 'image_raw']
    for e in endings:
        if topic.endswith(e):
            return topic[:-1 * len(e)]
    raise Exception("invalid topic for truncation %s looking for endings %s" %
                    (topic, endings))


class EncodingCache:
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
    with rosbag.Bag(bagfile_in) as bag:
        with rosbag.Bag(bagfile_out, 'w') as outbag:
            for topic, msg, t in bag.read_messages():
                if topic.endswith('image_raw'):
                    print("compressing %s, time is %s" % (topic, t))
                    bname = image_topic_basename(topic)
                    try:
                        msg, encoding_msg = compress_image(msg)
                        encoding_topic = bname + "encoding"
                        topic = bname + "compressed"
                        outbag.write(encoding_topic, encoding_msg, t)
                    except Exception as ex:
                        print("Exception: %s when parsing msg." % ex)
                # print("%s" % output_msg)
                outbag.write(topic, msg, t)


def uncompress(bagfile_in, bagfile_out):
    with rosbag.Bag(bagfile_in) as bag:
        with rosbag.Bag(bagfile_out, 'w') as outbag:
            encoding_cache = EncodingCache()
            for topic, msg, t in bag.read_messages():
                bname = image_topic_basename(topic)
                if topic.endswith('encoding'):
                    print("capturing encoding on %s: %s" %
                          (bname, msg.data))
                    encoding_cache.insert_encoding(bname, msg.data)
                    continue  # do not rewrite the encoding message
                if topic.endswith('compressed'):
                    print("uncompressing %s, time is %s" % (topic, t))
                    try:
                        enc = encoding_cache.lookup_encoding(bname)
                        msg = uncompress_image(msg, enc)
                        topic = bname + 'image_raw'
                    except Exception as ex:
                        print("Exception: %s when parsing msg." % ex)
                        raise

                # print("%s" % output_msg)
                outbag.write(topic, msg, t)


def main():
    compress(bagfile_name, output_bagfile_name)
    uncompress(output_bagfile_name, uncompressed_bagfile_name)

if __name__ == '__main__':
    main()
