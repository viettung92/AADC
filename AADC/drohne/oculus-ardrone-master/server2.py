# AR Drone control dummy server
# Copyright (C) 2015 Florian Jung
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

import cv2
import os
import socket
import sys
import time
import struct






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



while True:
    # Wait for a connection
    print >>sys.stderr, 'waiting for a connection'
    connection, client_address = sock.accept()
    conn2 = connection.makefile()
    try:
        print >>sys.stderr, 'connection from', client_address
        
        cap = cv2.VideoCapture("flight.avi")
        logfile = open("flight.log", "r")
        while True:
            data = conn2.readline()
            if data:
                if data=="get\n":
                    status, frame = cap.read()
                    values = logfile.readline().split()
                    phi = float(values[0])
                    theta = float(values[1])
                    psi = float(values[2])
                    batt = 100.0

                    framestr = frame.tostring()
                    #cv2.imshow("server", frame)
                    #cv2.waitKey(1)
                    lenframestr=len(framestr)
                    connection.sendall(struct.pack(">i",lenframestr)+framestr+struct.pack("@dddd", phi, theta, psi, batt));
                elif data[0:3] == "fly" and data[-1]=="\n":
                    values = data[3:-1].split()
                    print "fly ",values
                else:
                    print "corrupted command: '"+data+"'"
            else:
                print >>sys.stderr, 'no more data from', client_address
                break
    except:
        print "Dingens!!11!1!!!"
    finally:
        # Clean up the connection
        connection.close()

