# AR Drone control server
# Copyright (C) 2015 Florian Jung
#
# All rights reserved. Licensed under the 3-clause-BSD-license as follows:
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
# 
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
# 
# 3. Neither the name of the copyright holder nor the names of its contributors
#    may be used to endorse or promote products derived from this software without
#    specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


# keyboard:
# 1: enable xy control
# 2: enable z control
# 3: enable rot control
# a: fast
# z: slow
# t: trim
# 
# 
# joystick:
# thumb: land
# all four thumbbuttons: reset
# left shoulder + right shoulder + 11: takeoff
# 11: readjust
# left shoulder: relative flight mode if held
# right shoulder: float, do not hover, if held.


import libardrone.libardrone as libardrone
import pygame
import cv2
import os
import socket
import sys
import threading
import time
import struct
import math
from math import sin,cos,tan

OVERRIDE_THRESHOLD=0.01

global_cmd_x = 0
global_cmd_y = 0
global_cmd_z = 0
global_cmd_rot = 0
global_cmd_hover =  False # TODO XXX


def putStatusText(img, text, pos, activated):
    cv2.putText(img, text, pos, cv2.FONT_HERSHEY_PLAIN, 1, (0,0,255) if activated else (127,127,127), 2 if activated else 1)

def encode_int(i):
    i = int(i)
    return chr( (i/(2**24))%256) + chr( (i/(2**16))%256 ) +\
            chr( (i/(2**8))%256) + chr(i%256)


class ServerThread(threading.Thread):
    def run(self):

        global global_cmd_x
        global global_cmd_y
        global global_cmd_z
        global global_cmd_rot
        global global_cmd_hover

        while True:
            # Wait for a connection
            print >>sys.stderr, 'waiting for a connection'
            connection, client_address = sock.accept()
            conn2=connection.makefile()
            #try:
            if True:
                print >>sys.stderr, 'connection from', client_address

                while True:
                    data = conn2.readline()
                    if data:
                        if data=="get\n":
                            lock.acquire()
                            framestr = global_frame.tostring()
                            lenframestr=len(framestr)
                            connection.sendall(struct.pack(">i",lenframestr)+framestr+struct.pack("@dddd", global_phi, global_theta, global_psi, global_batt));
                            lock.release()
                        elif data[0:3] == "fly" and data[-1]=="\n":
                            values = data[3:-1].split()
                            lock.acquire()
                            global_cmd_x = float(values[0])
                            global_cmd_y = float(values[1])
                            global_cmd_z = float(values[2])
                            global_cmd_rot = float(values[3])
                            global_cmd_hover =  False # TODO XXX
                            lock.release()
                            print >>sys.stderr, "fly x/y/z/r/hov=",global_cmd_x,",",global_cmd_y,",","global_cmd_z",",",global_cmd_rot,",",global_cmd_hover
                        else:
                            print >>sys.stderr, "got corrupted command: '"+data+"'"
                    else:
                        print >>sys.stderr, 'no more data from', client_address
                        break
            #except:
            #    print "Dingens!!11!1!!!"
            #finally:
                # Clean up the connection
                connection.close()





server_address = '/home/flo/uds_socket'
try:
    os.unlink(server_address)
except OSError:
    if os.path.exists(server_address):
        raise

# Create a UDS socket
sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)

# Bind the socket to the port
print >>sys.stderr, 'starting up on %s' % server_address
sock.bind(server_address)

# Listen for incoming connections
sock.listen(1)

no_flight = False

try:
    pygame.init()
    pygame.joystick.init()
    js=pygame.joystick.Joystick(0)
    js.init()
    js_angle_shift = 0.0
except:
    print "no joystick! disabling flight controls"
    no_flight = True

manual_override_xy = True
manual_override_z = True
manual_override_rot = True

drone = libardrone.ARDrone(True, True)
drone.reset()


serverthread=ServerThread()
lock=threading.Lock()


writer = cv2.VideoWriter("flight.avi",cv2.VideoWriter_fourcc(*'MP42'),25,(1280,720),1)
logfile = open("flight.log", "w")


global_phi = 0.
global_theta = 0.
global_psi = 0.
global_batt = 0.
global_frame =  None

serverthread.start()

drone.set_max_vz(750.0000)
drone.set_max_rotspeed(1.0)
drone.set_max_angle(0.1)
limit_vz=750.0
limit_rot=1.0
limit_tilt=0.1

while True:
    if no_flight == False:
        pygame.event.pump()

        btn_leftshoulder =  js.get_button(4) or js.get_button(5)
        btn_rightshoulder = js.get_button(6) or js.get_button(7)
        btn_thumb = js.get_button(0) or js.get_button(1) or js.get_button(2) or js.get_button(3)
        btn_all = js.get_button(0) and js.get_button(1) and js.get_button(2) and js.get_button(3)
        btn_readjust = js.get_button(8)
        btn_unlock = js.get_button(9)

        if btn_thumb:
            drone.land()
            print "landing"
            manual_override_xy = True
            manual_override_z = True
            manual_override_rot = True
        if btn_leftshoulder and btn_rightshoulder and js.get_button(8):
            drone.takeoff()
            print "taking off"
            manual_override_xy = True
            manual_override_z = True
            manual_override_rot = True
        if btn_all:
            print "resetting"
            drone.reset()
            manual_override_xy = True
            manual_override_z = True
            manual_override_rot = True
        if btn_readjust:
            js_angle_shift = drone.navdata.get(0, dict()).get('psi',0)
        if btn_unlock:
            manual_override_rot = False

        rel_angle = (drone.navdata.get(0, dict()).get('psi',0) - js_angle_shift)/180.*math.pi

        js_x = js.get_axis(0)
        js_y = js.get_axis(1)
        js_z = -js.get_axis(4)
        js_rot = js.get_axis(3)
        js_radius =  math.sqrt(js_x**2 + js_y**2)
        if btn_leftshoulder==0:
            js_x, js_y =  ( js_x * cos(rel_angle) + js_y * sin(rel_angle) )   ,  ( -js_x * sin(rel_angle) + js_y * cos(rel_angle) )

        js_hover = (btn_rightshoulder==0 and (js_radius <= 0.01))

        if (js_radius > OVERRIDE_THRESHOLD): manual_override_xy = True
        if (abs(js_z) > OVERRIDE_THRESHOLD): manual_override_z = True
        if (abs(js_rot) > OVERRIDE_THRESHOLD): manual_override_rot = True

        if manual_override_xy:
            actual_hover, actual_x, actual_y = js_hover, js_x, js_y
        else:
            actual_hover, actual_x, actual_y = global_cmd_hover, global_cmd_x/limit_tilt, global_cmd_y/limit_tilt
            actual_x=max(-1.0, min(1.0, actual_x))
            actual_y=max(-1.0, min(1.0, actual_y))

        if manual_override_z:
            actual_z = js_z
        else:
            actual_z = global_cmd_z / limit_vz
            actual_z=max(-1.0, min(1.0, actual_z))

        if manual_override_rot:
            actual_rot = js_rot
        else:
            actual_rot = global_cmd_rot / limit_rot
            actual_rot=max(-1.0, min(1.0, actual_rot))

        drone.move_freely( not actual_hover  , actual_x, actual_y, actual_z, actual_rot) 
    
    
    
    lock.acquire()
    rawimg = drone.get_image()
    global_frame = cv2.cvtColor(rawimg, cv2.COLOR_BGR2RGB)
    global_phi = drone.navdata.get(0, dict()).get('phi',1337)
    global_theta = drone.navdata.get(0, dict()).get('theta',1337)
    global_psi = drone.navdata.get(0, dict()).get('psi',1337)
    global_batt = drone.navdata.get(0, dict()).get('batt',1337)
    lock.release()

    smallframe = cv2.resize(global_frame, (640,480))
    cv2.rectangle(smallframe, (0,0), (640,30), (255,255,255), -1)
    cv2.putText(smallframe, "override", (0,20), cv2.FONT_HERSHEY_PLAIN, 1, (255,0,0))
    putStatusText(smallframe, "XY", (100,20), manual_override_xy)
    putStatusText(smallframe, "height", (200,20), manual_override_z)
    putStatusText(smallframe, "rotation", (300,20), manual_override_rot)
    #cv2.putText(smallframe, "XY", (100,20), cv2.FONT_HERSHEY_PLAIN, 1, (0,0,255) if manual_override_xy else (127,127,127))
    #cv2.putText(smallframe, "height", (250,20), cv2.FONT_HERSHEY_PLAIN, 1, (0,0,255)   f manual_override_z else (127,127,127))
    #cv2.putText(smallframe, "rotation", (400,20), cv2.FONT_HERSHEY_PLAIN, 1, (0,0,255) if manual_override_rot else (127,127,127))
    cv2.imshow("frame", smallframe)
    writer.write(global_frame)
    logfile.write(str(global_phi)+"\t"+str(global_theta)+"\t"+str(global_psi)+"\n")
    logfile.flush()



    key = cv2.waitKey(10) & 0xFF

    if key == ord("t"):
        drone.trim()

    if key == ord("z"):
        drone.set_max_vz(750.0000)
        drone.set_max_rotspeed(10)
        drone.set_max_angle(0.2)
        limit_vz=750.0
        limit_rot=10.
        limit_tilt=0.2
        print "slow"
    elif key == ord("a"):
        drone.set_max_vz(10000.0000)
        drone.set_max_rotspeed(10)
        drone.set_max_angle(0.6)
        limit_vz=10000.0
        limit_rot=10.
        limit_tilt=0.6
        print "fast"
    
    if key == ord("1"):
        manual_override_xy = False
        print "yoooo"
    elif key == ord("2"):
        manual_override_z = False
    elif key == ord("3"):
        manual_override_rot = False


