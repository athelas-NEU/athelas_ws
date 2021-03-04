#!/usr/bin/env python3

import queue
from matplotlib.animation import FuncAnimation
import matplotlib.pyplot as plt
import numpy as np

import rospy
from steth_node.msg import biodatat
import time as t

rospy.init_node('main_node_fake', anonymous=True)
rate = rospy.Rate(10) # 10hz

def calculate_shift(data):
	if type(data) == float:
		return 1
	else:
		return len(data)

def print_list(_list):
	print("data: " + str(type(_list)) + ", " + str(len(_list)))

def plot_callback(frame, state):
	"""
	Fields needed from BioData:
		state.data_q
		state.plotdata
		state.lines
	"""

	while True:
		try:
			data_np = np.asarray(state.data_q.get_nowait())
			data = np.expand_dims(data_np, axis=1)
			# print_list(data)
		except queue.Empty:
			break

		shift = calculate_shift(data)
		state.plotdata = np.roll(state.plotdata, -shift, axis=0)
		 # Elements that roll beyond the last position are re-introduced

		# npdata = np.asarray(data)[0:shift]
		state.plotdata[-shift:,:] = data

	for column, line in enumerate(state.lines):
		# print(state.plotdata[:,column])
		line.set_ydata(state.plotdata[:,column])

	state.ax.autoscale_view(tight=None, scalex=False, scaley=True)
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

		window = 1000
		self.downsample = 1
		interval = 30 # this is update interval in miliseconds for plot (less than 30 crashes it :(
		samplerate = 44100.0
		channels = [1]
		length = int(window * samplerate / 1000 * self.downsample)

		self.data_q = queue.Queue()

		print("Sample Rate found: ", samplerate)

		self.plotdata = np.zeros((length,len(channels)))
		print("plotdata shape found: ", self.plotdata.shape)
		self.fig, self.ax = plt.subplots(figsize=(8,4))

		self.lines = self.ax.plot(self.plotdata,color = (0,1,0.29))

		self.ax.set_title("Athelas: " + str(self.sensor) + " data")
		self.ax.set_facecolor((0,0,0))
		self.ax.set_yticks([0])
		self.ax.yaxis.grid(True)

		self.start_time = t.time()
		self.sub = rospy.Subscriber(f"biosensors/{topic}", biodatat, self.__bio_callback)

		self.ani = FuncAnimation(self.fig, plot_callback, fargs=(self,), interval=interval, blit=True)
		plt.show()

		### End pyplot setup

		print("finished state init")

	def __bio_callback(self, data):
		datalist = data.data
		self.data_q.put(datalist)


myState = FakeBioState("stethoscope")    
