
# swarm_slam_demo
**Author**: Wang Kuan  
**Video**: [swarm_slam_demo](https://www.youtube.com/watch?v=SM4inr8sCjk)  
<a href="https://www.youtube.com/watch?v=SM4inr8sCjk" target= "blank "><img src="https://i.ytimg.com/vi/SM4inr8sCjk/hqdefault.jpg?custom=true&w=196&h=110&stc=true&jpg444=true&jpgq=90&sp=67&sigh=s6ickcZ8L180N7S1GjiM5TaoC14"
alt="swarm_slam_demo" width="240" height="180" border="10" /></a>


Swarm_slam_demo is a set of ROS nodes for running real-time multi-robot co-SLAM system, the main source of the SLAM is [SwarmSLAM](https://github.com/THUKey/SwarmSlam), in which I adapt the SLAM system proposed in [ORB-SLAM2](https://github.com/raulmur/ORB_SLAM2) and extend it for the use with multi-robot systems and the tool for saving map.  

## swarm_slam_node
- This node demonstrates how to use the [SwarmSLAM](https://github.com/THUKey/SwarmSlam) with multi-robot system:  
```
ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::Stystem::MONOCULAR,true,false,4);
```  
If you want to add (or reduce) the amount of the robots in the system, you can change the last parameter(default is `4`) of the System constructor
```
ros::Subscriber sub = nodeHandler.subscribe("/camera/image_raw", 1, &ImageGrabber::GrabImage,&igb);  
```
then change the Subscriber's topic `/camera/image_raw` to you own camera topic and write the Callback function like `ImageGrabber::GrabImage`
```
cv::Mat Tcw = mpSLAM->TrackMonocular(cv_ptr->image,2,cv_ptr->header.stamp.toSec(),CurrentKeyFramePtr);
```
in Callback function, change the second parameter of function TrackMonocular() to the id of robot(camera), now the new robot is added  

- In [SwarmSLAM](https://github.com/THUKey/SwarmSlam), you could find the source file of the multi-robot-slam system.In this system, I use multithreading to do Traking, each robot corresponds one Tracking thread which do the image process as well as Tracking. The KeyFrames those Tracking threads generate will be insert in KeyFrameDatabase and be used by Loopclosure thread and localmapping thread.  




## share_frame_node
- This node demonstrates how tho share the useful frame data each robot extracts from their Tracking thread:  
```
igb.pub_a_ = nodeHandler.advertise<ORB_SLAM2::frame>("frameinfo1",10);  
```
`ORB_SLAM2::frame` is the frame.msg I design for share the frame data, you can change it to `KeyPoint`,`Descripter`,etc. for your specified purpose, the types of those is declare in `ORB_SLAM2/msg`. You can also change the `frameinfo1` to the topic name you want.   
```
void ImageGrabber::PublishFrameinfo(ORB_SLAM2::Frame* CurrentKeyFramePtr);  
```
In addition, a pointer like `CurrentKeyFramePtr` of the `Frame` or `KeyFrame` you want to share is need, you can realize the publish function like this function
```
ros::Subscriber sub_a = nodeHandler.subscribe("/frameinfo0", 1, &ImageGrabber::insertKeyFrame,&igb);  
```
If you want to subscribe a other robots' frame data, you can declare the Subscriber like this, `/frameinfo0` is the topic you want to subscribe, and `insertKeyFrame` is the Callback function, the default function is inserting KeyFrame in local map, you can change it on you own propose.  

- For distributed use, I design a ROS message called `frame.msg` to share the `Frame` or `KeyFrame` each robot generate, one robot can publish their own frame data(Pose,KeyPoint,MapPoint,Descripter,etc.) to a specified topic, and subscribe the frame data other robots publish to assistant Mapping as well as Loopclosure.  

## save_map_node
- This node demonstrates how to use this function for saving the Map as binary file, and how to reuse the Map:  
```
SLAM.SaveMap("Slam_latest_Map.bin");
```
You can change `Slam_latest_Map.bin` to the filename you want, and every time you shut the node the binary Map will be save with that name in `/home/username/.ros/`  
```
ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true, bReuseMap, 1);  
```
the fifth parameter `bReuseMap` is a bool variable, if you want to reuse the Map, set it to `true`.  

- A major advantage of the multi-robot system is quickly Mapping. I implement the Map Saving function to this system, So that we can quickly explore a large area and Maping then save the Map for navigation mission.
- [MathewDenny](https://github.com/MathewDenny/ORB_SLAM2)'s code is helpful for me to use Boost serialization tool to implement this function  
