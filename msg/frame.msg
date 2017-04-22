Header header

#this is the msg for Frame info

#for Sharing Map
int32 mnMachineId

#Frame timestamp.
float64 mTimeStamp

#Number of KeyPoints.
int32 N

## Vector of keypoints (original for visualization) and undistorted (actually used by the system).
## In the stereo case, mvKeysUn is redundant as images must be rectified.
## In the RGB-D case, RGB images can be distorted.
#std::vector<cv::KeyPoint> mvKeys, mvKeysRight
keypoint[] mvKeysUn

## Corresponding stereo coordinate and depth for each keypoint.
## "Monocular" keypoints have a negative value.
#floatVector mvuRight
#std::vector<float> mvDepth

# Bag of Words Vector structures.
bow[] mBowVec
feature[] mFeatVec

# ORB descriptor, each row associated to a keypoint.
descriptor[] mDescriptors

## MapPoints associated to keypoints, NULL pointer if no association.
#std::vector<MapPoint*> mvpMapPoints
# Flag to identify outlier associations.
bool[] mvbOutlier

# Camera pose.
float32Vector[] mTcw

# Current and Next Frame id.
int32 nNextId
int32 mnId
