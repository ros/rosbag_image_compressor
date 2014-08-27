import rosbag

num_msgs = 100

with rosbag.Bag('head.bag', 'w') as outbag:
    for topic, msg, t in rosbag.Bag('raw.bag').read_messages():
        if num_msgs < 1:
            break
        num_msgs -= 1
        # This also replaces tf timestamps under the assumption
        # that all transforms in the message share the same timestamp
        if topic == "/tf" and msg.transforms:
            outbag.write(topic, msg, msg.transforms[0].header.stamp)
        else:
            outbag.write(topic, msg, msg.header.stamp if msg._has_header else t)
