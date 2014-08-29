import rosbag

num_msgs = 100

with rosbag.Bag('head.bag', 'w') as outbag:
    for topic, msg, t in rosbag.Bag('raw.bag').read_messages():
        if num_msgs < 1:
            break
        num_msgs -= 1
        outbag.write(topic, msg, t)
