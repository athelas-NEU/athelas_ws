#!/usr/bin/env python3

import argparse
import sys

from steth_node.msg import biodatat
import rospy
import random

def int_or_str(text):
    """Helper function for argument parsing."""
    try:
        return int(text)
    except ValueError:
        return text

parser = argparse.ArgumentParser(add_help=False)
parser.add_argument(
    '-n', '--number', action='store_true', default = 95,
    help='the number to publish')
parser.add_argument(
    '-d', '--deviation', type=int_or_str, default=5,
    help='the max deviation to randomly apply')
parser.add_argument(
	'-r', '--rate', type=int_or_str, default=10,
	help='the rate to run the ros node at')
args, remaining = parser.parse_known_args()

# We will use an audio call back function to put the data in queue
rospy.init_node('steth_publisher', anonymous=True)
rate = rospy.Rate(10) # 10hz

pub_fake = rospy.Publisher('/biosensors/fake', biodatat, queue_size=10)
pub_msg = biodatat()

try:
	print('#' * 80)
	print('press Ctlr+C to quit')
	print('#' * 80)
	while True:
		dev = random.randrange(-args.deviation, args.deviation)
		pub_msg.data = [args.number + dev]
		pub_fake.publish(pub_msg)
except KeyboardInterrupt:
	parser.exit('\nInterrupted by user')