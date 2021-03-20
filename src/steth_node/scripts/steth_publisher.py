#!/usr/bin/env python3

import argparse
import queue
import sys

import numpy as np
import sounddevice as sd

from std_msgs.msg import Float32MultiArray
import rospy
import time as t

def int_or_str(text):
    """Helper function for argument parsing."""
    try:
        return int(text)
    except ValueError:
        return text

parser = argparse.ArgumentParser(add_help=False)
parser.add_argument(
    '-l', '--list-devices', action='store_true',
    help='show list of audio devices and exit')
parser.add_argument(
    '-d', '--device', type=int_or_str, default=15,
    help='input device (numeric ID or substring)')
args, remaining = parser.parse_known_args()
if args.list_devices:
    print(sd.query_devices())
    parser.exit(0)

# Lets define audio variables
# We will use the default PC or Laptop mic to input the sound

device = args.device # id of the audio device by default
downsample = 1 # how much samples to drop
channels = [1] # a list of audio channels
interval = 30 # this is update interval in miliseconds for plot

# lets make a queue
q = queue.Queue()
# Please note that this sd.query_devices has an s in the end.
device_info =  sd.query_devices(device, 'input')
samplerate = device_info['default_samplerate']

# lets print it 
print("Sample Rate: ", samplerate)

# We will use an audio call back function to put the data in queue
rospy.init_node('steth_publisher', anonymous=True)
rate = rospy.Rate(10) # 10hz


QUEUE_SIZE = 100
pub_steth = rospy.Publisher('/biosensors/stethoscope', Float32MultiArray, queue_size=QUEUE_SIZE)
pub_msg = Float32MultiArray()


def audio_callback(indata,frames,time,status):
    sample = indata[::downsample,[0]]
    if len(sample) < 1:
        return

    pub_msg.data = list(sample)
    pub_steth.publish(pub_msg)
    print("length: " + str(len(pub_msg.data)))

    # print("Callback: " + str(type(sample)) + "," + str(sample[0]))

    # n_msgs = min(QUEUE_SIZE, len(sample))

    # for ii in range(0, n_msgs):
    #     pub_msg.time = float(t.time())
    #     pub_msg.data = float(sample[ii])
    #     pub_steth.publish(pub_msg)
    #     print(pub_msg)
    #     rate.sleep()

""" INPUT FROM MIC """
with sd.InputStream(device=device,
                   samplerate=samplerate,
                   channels=max(channels),
                   callback=audio_callback):
    rospy.spin()
    print("shutting down")

