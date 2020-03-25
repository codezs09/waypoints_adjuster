#!/usr/bin/env python

# this script demonstrates a simple algorithm for velocity settings

from wpProcess import WaypointProcessor
import matplotlib.pyplot as plt
import numpy as np

CSV_IMPORT_FILE = 'tmp_lindenwp_post.csv'

# constants defined below
AX_MAX = 1.0
AY_MAX = 2.0
V_HI = 20.0
V_LO = 3.0
CURV_THR = 0.05		# curvature threshold for low speeds



def setVel( station, v_hi, a_max ):
	n = len(v_hi)
	v_set = v_hi 
	l = 0
	r = 0
	for r in range( n ):
		if ( r==n-1 or v_set[r]<v_set[r+1] ):
			# left sweep
			for i in range( l-1,-1,-1 ):
				a_l = ( v_set[i+1]-v_set[i] ) / ( station[i+1]-station[i] )
				if ( a_l >= -a_max ):
					break; 
				else:
					v_set[i] = v_set[i+1]+a_max*( station[i+1]-station[i] ); 

			# right sweep
			for i in range( r+1, n ):
				a_r = ( v_set[i]-v_set[i-1] ) / ( station[i]-station[i-1] ); 
				if ( a_r <= a_max ):
					break;
				else:
					v_set[i] = v_set[i-1]+a_max*( station[i]-station[i-1] ); 

		elif ( v_set[r]>v_set[r+1] ):
			l = r+1

	return v_set


# 
def calAcc( station, vel ):
	s_ = np.asarray(station)
	v_ = np.asarray(vel)
	ds_ = np.gradient(s_)
	dv_ = np.gradient(v_)
	return dv_/ds_



# 
WaypointProcessor()
wp = WaypointProcessor()
wp.load_waypoints( CSV_IMPORT_FILE )
wp.update_curvature() 


# intial vel
v_hi = [ V_HI for i in range(len(wp.station)) ]
for i in range( len(wp.station) ):
	v_hi[i] = V_HI if abs(wp.curvature[i])<CURV_THR else V_LO

# 
vel = setVel( wp.station, v_hi, AX_MAX )
acc = calAcc( wp.station, vel )

# plot velocity wrt station 
plt.figure(1)
plt.plot( wp.station, v_hi, 'r-' )
plt.plot( wp.station, vel, 'b.')
plt.xlabel('staion [m]')
plt.ylabel('v [m/s]')
plt.legend(['v_hi','v'], loc='best')

# plot curvature wrt station 
plt.figure(2)
plt.plot( wp.station, wp.curvature, 'b.' )
plt.xlabel('station [m]')
plt.ylabel('curvature')

# plot acceleration 
plt.figure(3)
plt.plot( wp.station, acc, 'b.' )
plt.xlabel('staion [m]')
plt.ylabel('a [m/s^2]')
plt.show()
