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
 * $Author:: schoen $   $Date:: 2016-01-29 #$
 **********************************************************************/

#ifndef CVMATH_H_
#define CVMATH_H_

#include <opencv2/opencv.hpp>
#include <cmath>
#include <algorithm>

#define degreesToRadians(angleDegrees) ((angleDegrees) * CV_PI / 180.0)
#define radiansToDegrees(angleRadians) ((angleRadians) * 180.0 / M_PICV_PI)

class CVMath {
public:
	class Line {
	public:
		cv::Point2f normal;
		cv::Point2f pointOnLine;
		cv::Point2f direction;
		float distance;

		Line() {
			distance = 0;
		}

		Line(cv::Point2f pointOnLine, cv::Point2f direction) {
			this->pointOnLine = pointOnLine;
			this->direction = direction;

			Line_VectorToHesseForm(&normal, &distance, pointOnLine, direction);
		}

		inline cv::Point2f GetPointFor_X(float x) {
			return cv::Point2f (x,(distance - normal.x * x) / normal.y);
		}

		inline cv::Point2f GetPointFor_Y(float y) {
			return cv::Point2f ((distance - normal.y * y) / normal.x, y);
		}
	};

	class LineSegment : public Line {

	public:
		cv::Point2f start;
		cv::Point2f end;
		float length;

		LineSegment() : Line() {
			length = 0;
		}

		LineSegment(cv::Point2f pointOnLine, cv::Point2f direction) : Line(pointOnLine, direction) {
			length = 0;
		}

		bool inline Valid() {
			return (start != end);
		}

		void inline Draw(cv::Mat *debugImage, cv::Scalar color) {
			cv::line(*debugImage, start, end, color, 2, 4);
		}

		void SetStartEnd(cv::Point2f start, cv::Point2f end) {
			this->start = start;
			this->end = end;

			pointOnLine = start;
			direction = end - start;

			length = cv::norm(direction);
			if(length != 0) {
				direction /= length;
			}

			Line_VectorToHesseForm(&normal, &distance, pointOnLine, direction);
		}

		void CropX(float xStart, float xEnd) {
			start = GetPointFor_X(xStart);
			end = GetPointFor_X(xEnd);
			length = cv::norm(end - start);
			pointOnLine = start;
		}

		void CropY(float yStart, float yEnd) {
			start = GetPointFor_Y(yStart);
			end = GetPointFor_Y(yEnd);
			length = cv::norm(end - start);
			pointOnLine = start;
		}
	};


	static float cos2(float value) {
		return (1 - 0.5 * value * value);
	}

	/**
	 * rotation matrix using right hand rule: [0; 1; 0] will be [0; 0; 1]
	 * @param angle
	 * @return
	 */
	static cv::Mat RotationX_3D_RAD (float angle) {
		cv::Mat rot = cv::Mat::eye(3, 3, CV_32FC1);
		float cosineAngle = cos(angle);
		float sineAngle = sin(angle);
		rot.at<float>(1,1) = cosineAngle;
		rot.at<float>(2,2) = cosineAngle;

		rot.at<float>(1,2) = - sineAngle;
		rot.at<float>(2,1) =   sineAngle;

		return rot;
	}

	static inline cv::Mat RotationX_3D_Degree (float angleInDegree) {
		return RotationX_3D_RAD(degreesToRadians(angleInDegree));
	}

	static cv::Mat Rotation_2D_RAD (float angle) {
		cv::Mat rot(2, 2, CV_32FC1);
		float cosineAngle = cos(angle);
		float sineAngle = sin(angle);
		rot.at<float>(0,0) = cosineAngle;
		rot.at<float>(1,1) = cosineAngle;

		rot.at<float>(0,1) = - sineAngle;
		rot.at<float>(1,0) =   sineAngle;

		return rot;
	}

	/**
	 * n and d are line parameters in hesse normal form.
	 * e and dir are line parameteres in vector form
	 * @param n
	 * @param d
	 * @param e
	 * @param dir
	 * @return
	 */
	static inline cv::Point2f IntersectionTwoLines(cv::Point2f n, float d, cv::Point2f e, cv::Point2f dir) {
		return e + (d - n.dot(e)) * dir / n.dot(dir);
	}

	static inline cv::Point2f IntersectionTwoLines(Line line1, Line line2, float * t1) {
		*t1 = (line2.distance - line2.normal.dot(line1.pointOnLine)) / line2.normal.dot(line1.direction);
		return line1.pointOnLine + *t1 * line1.direction;
	}

	/**
	 * n and d are line parameters in hesse normal form.
	 * e and dir are line parameteres in vector form
	 * @param n output normal
	 * @param d output distance from (0,0) in units of cv::norm(n)
	 * @param e
	 * @param dir
	 * @return
	 */
	static inline void Line_VectorToHesseForm(cv::Point2f *n, float *d, cv::Point2f e, cv::Point2f dir) {
		*n = RotateCW90(dir);
		*d = e.dot(*n);
	}

	/**
	 * rotate vector clockwise 90 degrees
	 * @param vector
	 * @return
	 */
	static inline cv::Point2f RotateCW90(cv::Point2f vector) {
		return cv::Point2f(vector.y, -vector.x);
	}

	static inline cv::Point2f RotateCounterCW90(cv::Point2f vector) {
		return cv::Point2f(-vector.y, vector.x);
	}

	static inline cv::Matx12f RotateCW90(cv::Matx12f vector) {
		return cv::Matx12f(vector.val[1], -vector.val[0]);
	}

	static inline cv::Matx12f RotateCounterCW90(cv::Matx12f vector) {
		return cv::Matx12f(-vector.val[1], vector.val[0]);
	}

	/**
	 * rotate vector clockwise
	 * @param vector
	 * @return
	 */
	static inline cv::Point2f RotateCW(cv::Point2f vector, float angle) {
		float cosineAngle = cos(angle);
		float sineAngle = sin(angle);

		return cv::Point2f(vector.x * cosineAngle + vector.y * sineAngle, -vector.x * sineAngle + cosineAngle * vector.y);
	}

	static inline cv::Point2f Rotate(cv::Point2f vector, float cosineAngle, float sineAngle) {
		return cv::Point2f(vector.x * cosineAngle + vector.y * sineAngle, -vector.x * sineAngle + cosineAngle * vector.y);
	}

	/**
	 * TODO not tested yet
	 * @param result array with two elements
	 * @param center
	 * @param radius
	 * @param line
	 * @return solution count
	 */
	static inline int IntersectCircle(cv::Point2f *result, cv::Point2f center, float radius, Line line) {
		float b = 2 * line.direction.dot(line.pointOnLine - center);
		cv::Point2f tmp = line.pointOnLine - center;
		float c = tmp.dot(tmp) - radius * radius;

		float discriminant = b * b - 4 * c;
		if(discriminant < 0) {
			return 0;
		}

		float t = 0.5 *(-b + cv::sqrt(discriminant));
		result[0] = line.pointOnLine + t * line.direction;

		if(discriminant == 0) {
			result[1] = result[0];
			return 1;
		}

		t = 0.5 *(-b - cv::sqrt(discriminant));
		result[1] = line.pointOnLine + t * line.direction;

		return 2;
	}
};

#endif /* CVMATH_H_ */
