
import numpy as np
from numpy import linalg as lg

def wpCostFunc(P, *args):
	"""define the function to minimize the objective function as expressed in 
	   Eq. (6), S. Thrun, etc. Stanley: The Robot that won the DARPA Grand Challenge,
	   page 679-680.
	   Inputs:
	   		- P: list in the form of [x1, y1, x2, y2, ..., xn, yn]
	     	- *args: 
	     		Q, the reference waypoints provided as list [xr1, yr1, ...]
	     		beta, the weight value between f1 and f2 for the cost function
	   Return:
	    	- The calculated cost function value
	"""
	Q, beta = args
	
	# calculate the first item: closeness to the reference
	f1 = 0
	for i in range(len(Q)):
		f1 += ( P[i]-Q[i] )**2

	# calculate the second item: misalignment penality
	f2 = 0
	n = len(P)/2	# number of waypoints to be optmized
	for k in range(2,n):
		p_kminus1 = ( P[2*(k-1)-2], P[2*(k-1)-1] )
		p_k = ( P[2*k-2], P[2*k-1] )
		p_kplus1 = ( P[2*(k+1)-2], P[2*(k+1)-1] )

		vec_cur = ( p_k[0]-p_kminus1[0], p_k[1]-p_kminus1[1] )
		vec_next = ( p_kplus1[0]-p_k[0], p_kplus1[1]-p_k[1] )
		
		f2 += np.dot(vec_cur, vec_next)/lg.norm(vec_cur)/lg.norm(vec_next)

	f = f1-beta*f2
	return f


def getwpCurvature(x_, y_):
	'''
	Obtain signed curvature from discrete waypoints (x, y) with the 
	equation as: K = (x'y''-x''y')/(x'^2+y'^2)^(3/2)
	Inputs:
		- (x, y), should be in order 
	'''
	x = np.asarray(x_)
	y = np.asarray(y_)

	dx = np.gradient(x)   #x'
	dy = np.gradient(y)   #y'

	d2x = np.gradient(dx)    #x''
	d2y = np.gradient(dy)    #y''

	curvature = (dx * d2y - d2x * dy) / (dx * dx + dy * dy)**1.5
	return curvature








