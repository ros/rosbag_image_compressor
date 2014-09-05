
from PIL import Image

import os
import unittest
import tempfile
import shutil

from sensor_msgs.msg import Image as sensorImage

import rosbag
import rospy

from rosbag_image_compressor import compress_image,\
    uncompress_image, compress, uncompress, EncodingCache,\
    image_topic_basename


def create_image():
    img_data = [1, 2, 3, 4, 5, 6, 7, 8, 9, 0] * 10
    img_data = "1234567890" * 10
    # not available on precise image = Image.frombytes('L', (10, 10), img_data)
    image = Image.fromstring('L', (10, 10), img_data)
    return image


def create_image_msg():
    img = create_image()
    im = sensorImage()
    im.header.frame_id = 'foobar'
    im.width, im.height = img.size
    im.encoding = 'foobar'
    im.is_bigendian = False  # TODO
    im.step = im.width
    im.data = img.tostring()
    return im


class TestBag():
    def __init__(self, num_images=1):
        self.num_images = num_images

    def __enter__(self):
        self.tempdir = tempfile.mkdtemp()
        self.filename = os.path.join(self.tempdir, 'test.bag')
        with rosbag.Bag(self.filename, 'w') as bagfile:
            for n in range(self.num_images):
                bagfile.write('/foo/bar/image_raw', create_image_msg(),
                              rospy.Time(n + 100))
        return self

    def __exit__(	self, exc_type, exc_value, traceback):
        shutil.rmtree(self.tempdir)


class TestClass(unittest.TestCase):

    def test_compress_uncompress(self):
        i = create_image_msg()
        ci, encoding_msg = compress_image(i)
        uci = uncompress_image(ci, encoding_msg.data)
        self.check_contents(i, uci)

    def check_contents(self, i, uci):
        self.assertEqual(i.header.frame_id, uci.header.frame_id)
        self.assertEqual(i.header.stamp, uci.header.stamp)
        self.assertEqual(i.width, uci.width)
        self.assertEqual(i.height, uci.height)
        self.assertEqual(i.step, uci.step)
        self.assertEqual(i.is_bigendian, uci.is_bigendian)
        self.assertEqual(i.encoding, uci.encoding)
        self.assertEqual(i.data, uci.data)

    def test_encoding_cache(self):
        cache = EncodingCache()
        cache.insert_encoding('topic1', 'enc1')
        self.assertEqual('enc1', cache.lookup_encoding('topic1'),
                         "Coorect encoding")
        try:
            cache.lookup_encoding('topic/2')
            self.assertFalse("Should have excepted")
        except:
            self.assertTrue(True)

        cache.insert_encoding('topic/2', 'enc2')
        self.assertEqual('enc2', cache.lookup_encoding('topic/2'),
                         "Coorect encoding")

    def test_image_topic_basename(self):
        self.assertEqual(image_topic_basename('foo/bar/image_raw'), 'foo/bar/')
        self.assertEqual(image_topic_basename('foo/bar/compressed'), 'foo/bar/')
        self.assertEqual(image_topic_basename('foo/bar/encoding'), 'foo/bar/')
        try:
            image_topic_basename('foo/bar/other')
            self.assertFalse("Should have thrown")
        except:
            self.assertTrue("Has thrown")

    def test_bag_unbag_empty(self):
        with TestBag(0) as bag_context:
            intermediate = os.path.join(bag_context.tempdir, 'intermediate.bag')
            output = os.path.join(bag_context.tempdir, 'output.bag')
            compress(bag_context.filename, intermediate)
            uncompress(intermediate, output)

    def test_bag_unbag(self):
        with TestBag(4) as bag_context:
            intermediate = os.path.join(bag_context.tempdir, 'intermediate.bag')
            output = os.path.join(bag_context.tempdir, 'output.bag')
            compress(bag_context.filename, intermediate)
            uncompress(intermediate, output)
            with rosbag.Bag(output, 'r') as bag:
                for topic, msg, t in bag:
                    self.check_contents(create_image_msg(), msg)
