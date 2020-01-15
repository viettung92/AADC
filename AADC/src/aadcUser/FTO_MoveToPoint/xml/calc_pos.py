import sys
import math
import numpy

from math import cos, sin, pi
from sys import argv


class bcolors:
    HEADER = '\033[95m'
    BLUE = '\033[94m'
    GREEN = '\033[92m'
    RED =  '\033[91m'
    YELLOW = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

def rad(phi):
	return phi * pi / 180

def rotate(pos,phi):
#	if phi == 0:
#		return pos
	if phi < 0:
		phi = -phi
		rad_phi = rad(phi)
		rotM = [[cos(rad_phi), -sin(rad_phi)], [sin(rad_phi), cos(rad_phi)]]
	else: 
		rad_phi = rad(phi)
		rotM = [[cos(rad_phi), sin(rad_phi)], [-sin(rad_phi), cos(rad_phi)]]
	return numpy.matmul(rotM, pos) 



diff = (float(len(sys.argv)) - 1.0) / 3.0
diff = math.floor(diff) * 3

if (len(sys.argv) - 1 != diff) or len(sys.argv) < 4:
	print bcolors.RED + "usage: <new_coord_phi now_x now_y>, ... <as many triples as you want points>" 
	print "need phi as local change from pos to pos, please type an int between 0 and 360"
	print "also type a position as x and y, in global car coord system" + bcolors.ENDC
	exit()

#std car rotation
num = 0
phi_offset = 90.0
global_phi = 0.0
old_pos = [[0.0],[0.0]]
print"-----------------------------------------"
for i in range (1,len(sys.argv),3):

	new_pos = [[float(argv[i+1])],[float(argv[i+2])]]

	pos = [[new_pos[0][0]-old_pos[0][0]],[new_pos[1][0] - old_pos[1][0]]]
	old_pos = new_pos
	pos = rotate(pos,global_phi)

	new_phi = int(argv[i])
	global_phi += new_phi 

	#pos rotation mat
	print "new trajectory point at num ", num , bcolors.BLUE + bcolors.BOLD
	print "phi: ", round((new_phi) * pi / 180,3)
	print "x:   ", round(pos[0],3)
	print "y:   ", round(pos[1],3), bcolors.ENDC
	print"-----------------------------------------"
	num += 1

#print "This is the name of the script: ", sys.argv[0]
#print "Number of arguments: ", len(sys.argv)
#print "The arguments are: " , str(sys.argv)


