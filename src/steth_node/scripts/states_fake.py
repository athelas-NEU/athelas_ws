#!/usr/bin/env python3

import queue
from matplotlib.animation import FuncAnimation
import matplotlib.pyplot as plt
import numpy as np

import rospy
from steth_node.msg import biodatat
import time as t

rospy.init_node('main_node_fake', anonymous=True)


def calculate_shift(data):
	if type(data) == float:
		return 1
	else:
		return len(data)

def plot_callback(frame, state):
	"""
	Fields needed from BioData:
		state.data_q
		state.plotdata
		state.lines
	"""
	while True:
		try:
			data = state.data_q.get_nowait()
		except queue.Empty:
			break

		shift = calculate_shift(data)
		plotdata = np.roll(state.plotdata, -shift, axis=0)
		 # Elements that roll beyond the last position are re-introduced

		# npdata = np.asarray(data)[0:shift]

		plotdata[-shift:,:] = data
	for column, line in enumerate(state.lines):
		line.set_ydata(state.plotdata[:,column])
	return state.lines

class FakeBioState(object):
	"""
	Collect bio data for 15 seconds.
	"""

	def __init__(self, sensor):
		# super().__init__()
		self.sensor = sensor
		# topic = self.BIOSENSOR_MAP[self.sensor]["topic"]

		print("in init")

		topic = "stethoscope"

		### Begin pylot setup

		self.start_time = t.time()
		self.sub = rospy.Subscriber(f"biosensors/{topic}", biodatat, self.__bio_callback)

		plt.ion()
		plt.show()
		rospy.spin()

		print("exiting state init")

	def __bio_callback(self, data):
		# print("biocallback: " + str(data.data))

		plt.plot(data.data, data.time, '*')
		# plt.axis("equal")
		plt.draw()
		plt.pause(0.00000000001)

		# self.data_q.put(float(data.data))


myState = FakeBioState("stethoscope")    
