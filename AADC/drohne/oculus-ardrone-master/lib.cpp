// Client library for AR Drone control server
// Copyright (C) 2015 Florian Jung
// All rights reserved. Licensed under the 3-clause-BSD-license as follows:
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice, this
//    list of conditions and the following disclaimer.
// 
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 
// 3. Neither the name of the copyright holder nor the names of its contributors
//    may be used to endorse or promote products derived from this software without
//    specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <string.h>
#include <sys/un.h>
#include <time.h>

#include "lib.h"

using namespace cv;

#define BUFSIZE 67108864

static void die(const char* msg){perror(msg); exit(1);}
static void suicide(const char* msg){ fprintf(stderr, "%s\n", msg); exit(1); }

static int read_completely(int fd, void* buf, size_t len)
{
	size_t n_read;
	for (n_read = 0; n_read < len;)
	{
		size_t tmp = read(fd, buf, len-n_read);
		n_read+=tmp;
		buf = ((char*)buf)+tmp;
	}
	return n_read;
}


DroneConnection::DroneConnection(const char* sockpath)
{
	struct sockaddr_un my_sockaddr;
	buffer = new unsigned char[BUFSIZE];

	my_sockaddr.sun_family=AF_UNIX;
	strcpy(my_sockaddr.sun_path, sockpath);
	int sockaddrlen = strlen(my_sockaddr.sun_path) + sizeof(my_sockaddr.sun_family);

	sockfd = socket(AF_UNIX, SOCK_STREAM, 0);
	if (sockfd == -1) die("socket");


	if (connect(sockfd, (struct sockaddr*) &my_sockaddr, sockaddrlen) == -1)
		die("connect");
}

DroneConnection::~DroneConnection()
{
	close(sockfd);
	delete [] buffer;
}

void DroneConnection::get(Mat& frame, navdata_t* nav)
{
	write(sockfd,"get\n",4);

	read_completely(sockfd, buffer, 4);
	int framelen = ((buffer[0]*256+buffer[1])*256+buffer[2])*256+buffer[3];
	if (framelen + sizeof(navdata_t) > BUFSIZE) suicide("buffer too small");
	
	read_completely(sockfd, buffer, framelen+sizeof(navdata_t));
	
	const navdata_t* navdata = (navdata_t*)(buffer + framelen);
	memcpy(nav, navdata, sizeof(*navdata));

	frame = Mat(720,1280,CV_8UC3, buffer);
}

void DroneConnection::fly(float x, float y, float z, float rot)
{
	char buf[100];
	int len = snprintf(buf, sizeof(buf), "fly %f %f %f %f\n", x,y,z,rot);
	if (len >= sizeof(buf)-1)
	{
		printf("ERROR: buffer too small in DroneConnection::fly()!\n");
		return;
	}
	printf("%s\n",buf);

	write(sockfd, buf, len);
}
