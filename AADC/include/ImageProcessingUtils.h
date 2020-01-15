/**
 * Copyright (c)
 * Audi Autonomous Driving Cup. TEAM FAUtonOHM.  All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
 * 3. All advertising materials mentioning features or use of this software must display the following acknowledgement: This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.
 * 4. Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS AS IS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 **********************************************************************
 * $Author:: schoen $   $Date:: 2016-02-05 #$
 **********************************************************************/

#ifndef IMAGEPROCESSINGUTILS_H_
#define IMAGEPROCESSINGUTILS_H_

#define BINARY_IMAGE_WIDTH 300
#define BINARY_IMAGE_HEIGHT 300
#define BINARY_DEBUG_IMAGE_HEIGHT 350
#define METER_TO_PIXEL(x) (200.0 * (x))
#define PIXEL_TO_METER(x) (0.005 * (x))

namespace ImageUtils {
	inline float MeterToPixel (float x) {
		return 200 * x;
	}

	inline float PixelToMeter(float x) {
		return 0.005 * x;
	}

	inline cv::Point2f ConvertToWorldCoordinates(cv::Point2f imageCoordinate, cv::Point2f frame) {
		cv::Point2f temp =  cv::Point2f(-imageCoordinate.y, -imageCoordinate.x) / 200.0;
		return (temp + frame);
	}

	inline cv::Point2f ConvertToImageCoordinates(cv::Point2f worldCoordinate, cv::Point2f frame) {
		cv::Point2f temp = 200*(worldCoordinate - frame);
		return cv::Point2f(-temp.y, - temp.x);
	}
}

#endif /* IMAGEPROCESSINGUTILS_H_ */
