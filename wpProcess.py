#!/usr/bin/env python

import csv
import numpy as np
from scipy import optimize as opt
from scipy.interpolate import interp1d
from scipy.signal import savgol_filter
import matplotlib.pyplot as plt
import math
import copy
from wp_funs import wpCostFunc, getwpCurvature


CSV_HEADER = ['x', 'y', 'z', 'yaw']
CSV_IMPORT_FILE = 'test_wp.csv'
CSV_EXPORT_FILE = CSV_IMPORT_FILE[:-4]+'_post.csv'

class WaypointProcessor():
	def __init__(self):
		self.x = []
		self.y = []
		self.z = []
		self.th = []
		self.station = []
		self.curvature = []

	def load_waypoints(self,fname):
		del self.x[:]
		del self.y[:]
		del self.z[:]
		del self.th[:]
		del self.station[:]
		with open(fname,'r') as wfile:
		    reader = csv.DictReader(wfile, CSV_HEADER)
		    for i, wp in enumerate(reader):
				self.x.append(float(wp['x']))
				self.y.append( float(wp['y']) )
				self.z.append( float(wp['z']) )
				self.th.append( float(wp['yaw']) )
				if i==0:
					self.station.append(0.0)
				else:
					self.station.append( self.station[-1]+ 
						math.sqrt( (self.x[-1]-self.x[-2])**2+
						           (self.y[-1]-self.y[-2])**2 ) )

	def scatter_waypoints(self,ds=1):	# ds- gap of station 
		s_ = [0.0]
		while s_[-1]+ds<self.station[-1]:
			s_.append(s_[-1]+ds)
		s_.append(self.station[-1])

		# interpolation
		fx = interp1d(self.station, self.x)
		fy = interp1d(self.station, self.y)
		fz = interp1d(self.station, self.z)
		fth = interp1d(self.station, self.th)
		x_ = [float(i) for i in fx(s_)]
		y_ = [float(i) for i in fy(s_)]
		z_ = [float(i) for i in fz(s_)]
		th_ = [float(i) for i in fth(s_)]

		# replace 
		self.x = x_
		self.y = y_
		self.z = z_
		self.th = th_
		self.station = s_

	def storecsv_waypoints(self,new_fname):
		state_zipped = zip(self.x, self.y, self.z, self.th)
		with open(new_fname,mode ='wb') as wfile:
		     writer = csv.writer(wfile)
		     writer.writerows(state_zipped)
		wfile.close()

	def getThFromxy(self):
		th_ = []
		for i in range(len(self.x)-1):
			th_i = math.atan2( self.y[i+1]-self.y[i],
							   self.x[i+1]-self.x[i] )
			th_.append( th_i )
		th_.append( th_i )	# heading for the last way point
		return th_

	def updateThFromxy(self):
		self.th = self.getThFromxy()

	def smooth_waypoints(self, beta = 10.0):
		# use optmization tool to smooth the path
		# opt
		P0 = []
		for i in range(len(self.x)):
			P0.append(self.x[i])
			P0.append(self.y[i])
		
		P_opt = opt.fmin_cg( wpCostFunc, P0, args=(P0, beta) )

		# update x, y, and th
		for i in range(len(self.x)):
			self.x[i] = P_opt[2*i]
			self.y[i] = P_opt[2*i+1]

		self.updateThFromxy()

	def update_curvature(self,ds=1):
		# suggested to use after scatter_waypoints() function
		curvature_ = getwpCurvature(self.x, self.y)
		# smooth the curvature before updating	
		tmp = int( math.floor(11/ds) )		# to ensure the span around 11 m
		winlen = tmp if tmp%2!=0 else (tmp+1)	# winlen should be odd
		self.curvature = savgol_filter(curvature_, window_length=winlen, polyorder=2).tolist()
		# debug comparison
		# plt.plot(self.station, curvature_, 'b.')
		# plt.plot(self.station, self.curvature, 'r.')
		# plt.xlabel('station [m]')
		# plt.ylabel('curvature')
		# plt.legend(['original','smoothed'], loc='best')
		# plt.show()

	def updatewp_density(self):
		'''
		Readjust waypoint density or distribution according to the curvature calculated from 
		the function update_curvature()
		'''
		ds = 1
		self.scatter_waypoints(ds)
		self.update_curvature(ds)

		x_ = []
		y_ = []
		station_ = []
		curvature_ = []

		x_.append(self.x[0])
		y_.append(self.y[0])
		idx_org = 0
		station_.append(self.station[0])
		curvature_.append(self.curvature[0])
		while station_[-1] < self.station[-1]:
			# obtain s, s.t. curvature vlaue is within limits
			ds = min( 16.0, self.station[-1]-station_[-1] )
			flg_runonce = False
			while not flg_runonce or ds>=1.0:
				flg_runonce = True
				s = station_[-1]+ds
				flg_excd = False	# flag indicator whether it exceeds curvature limit
				coef = 0.1

				# check the curvature in the range [station_[-1], station_[-1]+ds] exceeding limit
				i = idx_org
				while i<len(self.station) and self.station[i]<=s:					
					if ds > 1.0 and abs(self.curvature[i]) > max(0.01, coef/ds):
						flg_excd = True
						break
					i += 1

				if flg_excd:
					ds /= 2
				else:
					break
			print ds
			idx_org = i
			# interp for new x, y, curvature at s
			fx = interp1d(self.station, self.x)
			fy = interp1d(self.station, self.y)
			fcurv = interp1d(self.station, self.curvature)
			x_.append( float( fx(s) ) )
			y_.append( float(fy(s)) )
			station_.append(s)
			curvature_.append( float(fcurv(s)) )

		# update the class members x, y, th, station, curvature
		# plt.plot(self.x, self.y, 'b.')
		# plt.plot(x_, y_, 'r.')
		# plt.xlabel('x [m]')
		# plt.ylabel('y [m]')
		# plt.legend(['original','adjusted'], loc='best')
		# plt.show()

		# plt.plot(self.station, self.curvature, 'b.')		
		# plt.plot(station_, curvature_, 'r.')
		# plt.xlabel('station [m]')
		# plt.ylabel('curvature [m]')
		# plt.legend(['original','adjusted'], loc='best')
		# plt.show()

		self.x = x_
		self.y = y_	
		self.updateThFromxy()
		self.station = station_
		self.curvature = curvature_


if __name__=='__main__':
	wp_raw = WaypointProcessor()
	wp_raw.load_waypoints(CSV_IMPORT_FILE)
	wp = copy.deepcopy(wp_raw)
	wp.updatewp_density()
	

	# smooth
	wp_smth = copy.deepcopy(wp)
	wp_smth.smooth_waypoints(beta=10)
	wp_smth.storecsv_waypoints(CSV_EXPORT_FILE)

	plt.plot(wp.x, wp.y, 'b.')
	plt.plot(wp_smth.x, wp_smth.y, 'r.')
	plt.xlabel('x [m]')
	plt.ylabel('y [m]')
	plt.axis('equal')
	plt.legend(['adjusted','optimized'], loc='best')
	plt.show()
	
	plt.plot(wp.station, [abs(i)*180.0/math.pi for i in wp.th], 'b.')
	plt.plot(wp_smth.station, [abs(i)*180.0/math.pi for i in wp_smth.th], 'r.')
	plt.xlabel('station [m]')
	plt.ylabel('th [deg]')
	plt.axis('equal')
	plt.legend(['adjusted','optimized'], loc='best')
	plt.show()
