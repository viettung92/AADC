#pragma once

#include <string>


using std::string;


#pragma pack(push,1)

struct mtp_properties
{
    float mtp_steeringAngle;
    float mtp_movementSpeed;
    float max_speed;
    float min_speed;
    float kalpha;
    float kbeta_t;
    float offset_steering;
    float krho_forw;
    float krho_back;
    float reduction_offset;
    float goalDistanceShutOff;

    bool  mtp_use_absolute_pos;
    bool  mtp_stop_after_command;
};


/*  //TEST CASE

    test_properties test_p;
    LoadXMLData("/home/aadc/AADC/utilities/propertyXMLs/test_properties.xml", (void*) (&test_p));
    LOG_SUCCESS(cString::Format("float: %f", test_p.f));
    LOG_SUCCESS(cString::Format("int  : %i", test_p.i));
    LOG_SUCCESS(cString::Format("int  : %i", test_p.i2));
    LOG_SUCCESS(cString::Format("uint : %i", test_p.ui));
    LOG_SUCCESS(cString::Format("bool : %i", test_p.b));
    LOG_SUCCESS(cString::Format("str  : >%s<", test_p.s.c_str()));
    LOG_SUCCESS(cString::Format("long : %i", test_p.l));
    LOG_SUCCESS(cString::Format("ulong: %i", test_p.ul));

 * */
struct test_properties{
    float f;
    int i;
    int i2;
    unsigned int ui;
    bool b;
    string s;
    long l;
    unsigned long ul;
};



struct usacc_properties{
        tInt32 m_propI32CounterThresholdNoMove;
        tInt32 m_propI32CounterThresholdMoveAgain;
	
        tFloat32 m_propF32LowerBoundSpeed;
        tFloat32 m_propF32SensorOffsetSteering;
        tFloat32 m_propF32SensorFrontCheckLimit;
        tFloat32 m_propF32ObjectThresholdAngle;
        tFloat32 m_propF32ObjectThresholdRadius;
        tFloat32 m_propF32ObjectThresholdDetection;
        tFloat32 m_propF32CloseObjectThresholdAngle;
        tFloat32 m_propF32wheelbase;
        tFloat32 m_propF32ThresholdVisualRangeBend;
        tFloat32 m_propF32ThresholdReducedVisualRangeOneSide;
        tFloat32 m_propF32OffsetParkingY;
        tFloat32 m_propF32DistanceToStopLine;
        tFloat32 m_propF32PixToMM;
        tFloat32 m_propF32CamOffsety;
        tFloat32 m_propF32XminOfRamp;
        tFloat32 m_propF32XmaxOfRamp;
        tFloat32 m_propF32IntersectionOncomingXmin;
        tFloat32 m_propF32IntersectionOncomingYmin;
        tFloat32 m_propF32IntersectionOncomingXmax;
        tFloat32 m_propF32IntersectionOncomingYmax;
        tFloat32 m_propF32IntersectionCrosstrafficRightXmin;
        tFloat32 m_propF32IntersectionCrosstrafficRightYmin;
        tFloat32 m_propF32IntersectionCrosstrafficRightXmax;
        tFloat32 m_propF32IntersectionCrosstrafficRightYmax;
        tFloat32 m_propF32IntersectionCrosstrafficLeftXmin;
        tFloat32 m_propF32IntersectionCrosstrafficLeftYmin;
        tFloat32 m_propF32IntersectionCrosstrafficLeftXmax;
        tFloat32 m_propF32IntersectionCrosstrafficLeftYmax;
        tFloat32 m_propF32OvertakeOncomingXmin;
        tFloat32 m_propF32OvertakeOncomingYmin;
        tFloat32 m_propF32OvertakeOncomingXmax;
        tFloat32 m_propF32OvertakeOncomingYmax;
        tFloat32 m_propF32OvertakeOriginallaneXmin;
        tFloat32 m_propF32OvertakeOriginallaneYmin;
        tFloat32 m_propF32OvertakeOriginallaneXmax;
        tFloat32 m_propF32OvertakeOriginallaneYmax;
        tFloat32 m_propF32OvertakeObstacleStraightXmin;
        tFloat32 m_propF32OvertakeObstacleStraightYmin;
        tFloat32 m_propF32OvertakeObstacleStraightXmax;
        tFloat32 m_propF32OvertakeObstacleStraightYmax;
        tFloat32 m_propF32OvertakeObstacleStraightLeftHalfXmin;
        tFloat32 m_propF32OvertakeObstacleStraightLeftHalfYmin;
        tFloat32 m_propF32OvertakeObstacleStraightLeftHalfXmax;
        tFloat32 m_propF32OvertakeObstacleStraightLeftHalfYmax;
        tFloat32 m_propF32ParkingCrossOncomingLaneXmin;
        tFloat32 m_propF32ParkingCrossOncomingLaneYmin;
        tFloat32 m_propF32ParkingCrossOncomingLaneXmax;
        tFloat32 m_propF32ParkingCrossOncomingLaneYmax;
        tFloat32 m_propF32ParkingPullOutCrossOncoming1Xmin;
        tFloat32 m_propF32ParkingPullOutCrossOncoming1Xmax;
        tFloat32 m_propF32ParkingPullOutCrossOncoming1Ymin;
        tFloat32 m_propF32ParkingPullOutCrossOncoming1Ymax;
        tFloat32 m_propF32CheckCrosswalkXmin;
        tFloat32 m_propF32CheckCrosswalkXmax;
        tFloat32 m_propF32CheckCrosswalkYmin;
        tFloat32 m_propF32CheckCrosswalkYmax;
        tFloat32 m_propF32OvertakeObstacleOnlyRightHandSideXmin;
        tFloat32 m_propF32OvertakeObstacleOnlyRightHandSideYmin;
        tFloat32 m_propF32OvertakeObstacleOnlyRightHandSideXmax;
        tFloat32 m_propF32OvertakeObstacleOnlyRightHandSideYmax;
        tFloat32 m_propF32ThresholdSlopeForRamp;
        tFloat32 m_propF32ThresholdDifferenzYNotOnLine;
        tFloat32 m_propF32MinLengthOfTheRamp;
        tFloat32 m_propF32MaxLengthOfTheRamp;
        tFloat32 m_propF32YDiffRamp;
        tFloat32 m_propF32XDiffMax;
        tFloat32 m_propF32MergeCheckTargetLaneXmin;
        tFloat32 m_propF32MergeCheckTargetLaneYmin;
        tFloat32 m_propF32MergeCheckTargetLaneXmax;
        tFloat32 m_propF32MergeCheckTargetLaneYmax;
        
	
        tBool m_propBLaserOutputDrive;
        tBool m_propBDebugFirstStart;
        tBool m_propBDebugROI;
        tBool m_propBDebugCurve;
        tBool m_propBDebugActionCommand;
        tBool m_propBDebugOvertakingROI;
        tBool m_propBDebugStraight;
        tBool m_propBDebugRamp;

        tUInt32 m_propUI32FreeROIBoundary;
        tUInt32 m_propUI32OccupiedROIBoundary;
        tUInt32 m_propUI32TemporaryOccupiedROIBoundary;
        tUInt32 m_propUI32EndOfForLoopToCheckROI;
        tUInt32 m_propUI32Frames;
        tUInt32 m_propROIPoint1X;
        tUInt32 m_propROIPoint1Y;
        tUInt32 m_propROIPoint2X;
        tUInt32 m_propROIPoint2Y;
        tUInt32 m_propROIPoint3X;
        tUInt32 m_propROIPoint3Y;
        tUInt32 m_propROIPoint4X;
        tUInt32 m_propROIPoint4Y;
        tUInt32 m_propTransformedImageWidth;
        tUInt32 m_propTransformedImageHeight;
        tUInt32 m_propui32MinNumOfPointsRamp;
        tUInt32 m_propui32NumberLaneDetectionInputForRamp;
};

struct isd_properties
{

tFloat32	m_propF32XOffset;
tFloat32	m_propF32DistStartSignDetection;
tFloat32	m_propF32DistCameraLS;
tFloat32	m_propF32AdjustmentFactor;
tFloat32	m_propF32DistStartLS;
tFloat32	m_propF32DistFaktor;
tFloat32	m_propF32MinAngle;
tFloat32	m_propF32MaxAngle;
tFloat32	m_propF32RadiusRange;
tFloat32	m_propF32XMax;
tFloat32	m_propF32XMin;
tFloat32	m_propF32YMax;
tFloat32	m_propF32YMin;
tFloat32	m_propF32XMinArea;
tFloat32	m_propF32XMaxArea;
tFloat32	m_propF32WidthSign;
tFloat32	m_propF32ThresholdXNoSign;
tInt32		m_propISampleCount;


};



struct intersection_detection_binh_properties
{
    unsigned int  m_propROIPoint1X;
    unsigned int  m_propROIPoint1Y;
    unsigned int  m_propROIPoint2X;
    unsigned int  m_propROIPoint2Y;
    unsigned int  m_propROIPoint3X;
    unsigned int  m_propROIPoint3Y;
    unsigned int  m_propROIPoint4X;
    unsigned int  m_propROIPoint4Y;
    unsigned int  m_propTransformedImageWidth;
    unsigned int  m_propTransformedImageHeight;
    unsigned int  m_propGaussianKernelSize;
    unsigned int  m_propBinaryThreshold;
    unsigned int  m_propHarrisBlockSize;
    unsigned int  m_propHarrisAperture;
    unsigned int  m_propHarrisThreshold;
    unsigned int  m_propLaneROIWidth;
    unsigned int  m_propFrame;

    bool m_propEnableGaussianBlur;
    bool m_propEnableBinaryThreshold;
    bool m_propEnableCornerHarris;
};


struct lanedetection_properties
{
    unsigned int  m_propROIPoint1X;
    unsigned int  m_propROIPoint1Y;
    unsigned int  m_propROIPoint2X;
    unsigned int  m_propROIPoint2Y;
    unsigned int  m_propROIPoint3X;
    unsigned int  m_propROIPoint3Y;
    unsigned int  m_propROIPoint4X;
    unsigned int  m_propROIPoint4Y;
    unsigned int  m_propTransformedImageWidth;
    unsigned int  m_propTransformedImageHeight;
    unsigned int  m_propGaussianKernelSize;
    unsigned int  m_propCannyHighThreshold;
    unsigned int  m_propCannyKernelSize;
    unsigned int  m_propDilationSize;
    unsigned int  m_propDilationPoint;
    unsigned int  m_propHoughThreshold;
    unsigned int  m_propHoughMinLineLength;
    unsigned int  m_propHoughMaxLineGap;
    unsigned int  m_propNoCannyThreshold;
    unsigned int  m_propFramesPerAction;

    int     m_propLaneAnglePointLeftX;
    int     m_propLaneAnglePointLeftY;
    int     m_propLaneAnglePointRightX;
    int     m_propLaneAnglePointRightY;
    int     m_propLaneMiddlePointY;
    int     m_propLaneMiddlePointILCY;
    int     m_propLaneMiddlePointIRCY;
    int     m_propLaneDistanceRight;
    int     m_propLaneDistanceRightILC;
    int     m_propLaneDistanceLeftIRC;
    int     m_propLaneDistanceMiddle;
    int     m_propLaneDistanceLimitRight;
    int     m_propLaneDistanceLimitLeft;
    int     m_propLaneDistanceRightFar;
    int     m_propLaneDistanceLeftFar;
    int     m_propSteeringWeakRightLimit;
    int     m_propSteeringHeavyRightLimit;
    int     m_propSteeringWeakLeftLimit;
    int     m_propSteeringHeavyLeftLimit;
    int     m_propCarPositionInImageX;
    int     m_propCarPositionInImageY;

    float m_propSteeringOffset;
    float m_propSteeringWeakRight;
    float m_propSteeringHeavyRight;
    float m_propSteeringWeakLeft;
    float m_propSteeringHeavyLeft;
    float m_propSteeringLaneOffsetRight;
    float m_propSteeringLaneOffsetLeft;
    float m_propSteeringLaneOffsetRightFar;
    float m_propSteeringLaneOffsetLeftFar;
    float m_propSpeedStraight;
    float m_propSpeedWeakCurve;
    float m_propSpeedHeavyCurve;
    float m_F32InCarViewXmin;
    float m_F32InCarViewXmax;
    float m_F32InCarViewYmin;
    float m_F32InCarViewYmax;

    bool    m_propEnableGaussianBlur;
    bool    m_propEnableCannyEdge;
    bool    m_propEnableDilation;
    bool    m_propEnableHough;
    bool    m_propEnableDebugging;
    bool    m_propEnableDebuggingVideo;
};

struct markerdetector_properties
{
tFloat32	m_f32MarkerSize;

tFloat32	m_propF32CalibZOffset;
tFloat32	m_propF32CalibZSlope;
tFloat32	m_propF32MaxDist;
tInt32	m_propI32ROIOffsetX;
tInt32	m_propI32ROIOffsetY;
tInt32	m_propI32ROIWidth;
tInt32	m_propI32ROIWidthCurve;
tInt32	m_propI32ROIHeight;

 
tInt32	m_propI32DropFrame;
};


struct speedcontroller_properties
{
tFloat64	m_f64PIDKp;
tFloat64	m_f64PIDKi;
tFloat64	m_f64PIDKd;
tFloat64	m_f64PIDSampleTime;
tFloat64	m_f64PIDMinimumOutput;
tFloat64	m_f64PIDMaximumOutput;
tFloat64	m_f64PT1OutputFactor;
tFloat64	m_f64PT1TimeConstant;
tFloat64	m_f64PT1CorrectionFactor;
tFloat64	m_f64PT1Gain;
tInt32	m_i32ControllerMode;
tInt32	m_i32AntiWindupMethod;
tBool	m_bShowDebug;

};

struct steeringcontroller_properties
{
tFloat64	m_f64PIDKp;
tFloat64	m_f64PIDKi;
tFloat64	m_f64PIDKd;
tFloat64	m_f64PIDSampleTime;
tFloat64	m_f64PIDMinimumOutput;
tFloat64	m_f64PIDMaximumOutput;
tFloat64	m_f64PT1OutputFactor;
tFloat64	m_f64PT1TimeConstant;
tFloat64	m_f64PT1CorrectionFactor;
tFloat64	m_f64PT1Gain;
tInt32	m_i32ControllerMode;
tInt32	m_i32AntiWindupMethod;
tBool	m_bShowDebug;

};

#pragma pack(pop)
