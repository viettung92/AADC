#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <string.h>
#include <sys/un.h>
#include <time.h>
#include <pthread.h>
#include <semaphore.h>
#include "ringbuf.h"
#include "regulator.h"
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

#define PI 3.141592654
#define SIZE 300

int mouse_x=0, mouse_y=0;

void onMouse(int event, int x, int y, int, void*)
{
	mouse_x = x;
	mouse_y = y;
}

void draw_angle(Mat& m, float angle, Scalar col)
{
	line(m, Point(SIZE/2,SIZE/2), Point(SIZE/2+cos(angle)*SIZE/3, SIZE/2+sin(angle)*SIZE/3), col);
}

float fixup_range(float a, float low, float upp)
{
	float tot=upp-low;
	while (a < low) a+=tot;
	while (a>= upp) a-=tot;
	return a;
}


int main()
{
	Mat visu;
	namedWindow("dingens");
	setMouseCallback("dingens", onMouse);
	int time_msec=0;
	float winkel_ist=0;
	Regulator regulator(1.0,0.0,1,0.0);
	Ringbuffer cmd_delay_queue(50); // 1 sec delay
	Ringbuffer lazyness_ringbuf(2);

	while(waitKey(10)&255 !='x')
	{
		visu=Mat::zeros(SIZE,SIZE,CV_32FC3);

		float winkel_soll = atan2(mouse_y-SIZE/2, mouse_x-SIZE/2);
		draw_angle(visu, winkel_soll, Scalar(0,255,0));
		
		regulator.put(fixup_range(winkel_soll-winkel_ist,-PI,PI), time_msec);
		cmd_delay_queue.put(regulator.get());
		
		
		lazyness_ringbuf.put(cmd_delay_queue.front());
		cout << winkel_soll << "\t" << winkel_ist << "\t" << cmd_delay_queue.front() << "\t" << regulator.get() <<endl;
		winkel_ist=fixup_range(winkel_ist+lazyness_ringbuf.get()/100., -PI,PI);
		
		draw_angle(visu, winkel_ist, Scalar(0,0,255));
		
		imshow("dingens", visu);

		time_msec+=10;
	}
}
