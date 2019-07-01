## dotnect_stack TODO
Beside the inline TODO mark:

#### dotnect_dgram_socket
###### high
- [ ] delete erase when sensornet finished
- [ ]
###### low
- [ ] blocking socket with fd state interrupt
- [ ]

#### dotnect_markers
###### high
- [ ]
###### low
- [ ] convert Int32ArrayPublisher to service
- [ ]

#### dotnect_platform
###### high
- [ ] dotnect_stack/dotnect_rviz_plugin/resource/manually: uic -o dotnect_window.h dotnect_windowDOTui
- [ ] container jaccessors i update names
- [ ] $ Val3FstreamSocket::Val3FstreamSocket(<1:sin_addr> <2:sin_port> <3:be_verbose:0/1(int)>)
//    <1:groupIP>    server-side socket to connect to.
//    <2:socketPort> server-side port.
- [ ] configi members are ints or strings?
- [ ] how are movej args structured, tool?
- [ ] accel, vel in sendmDesc ok
- [ ] gui from .ui file and designed in qtcreator -> ui_header goes into .h of .cpp implementation
new package or this one? this one probably: all data here, can still be seperated
- [ ] panel
 printf("<arg2>  1: beVerbose x,y,z,u,v,depth  0: dont beVerbose; (Intrinsics are hardcoded)\n");
 string sensor_topics[7] = {
   [0] = "enter your own",
   [1] = "/camera/depth/image_raw",
   [2] = "/camera/depth/image_rect_raw",
   [3] = "/kinect2/sd/image_depth",
   [4] = "/kinect2/qhd/image_depth_rect",
   [5] = "/kinect2/hd/image_depth_rect",
   [6] = "",                     // user input topic
- [ ] scaling ofs
- [ ] send enable disable menu with error throttle function
add to readme
1. $ roslaunch dotnect_launch dotnect.launch
2. $ feed topic, so this goes away 'dotnect_platform: empty /dotnect_dgram_socket/rot'
3. $ hit ENTER
4. $ feed topic, so this goes away 'kinect_tracking: no images dropping in on /...'
5. $ hit the red ball
- [ ] fix license
- [ ] findRefFrame2 not called
- [ ]
###### low
- [ ] getGenerateVal3()->sendMovej not called
- [ ] configi members are ints or strings?
how are movej args structured, tool?   
accel, vel in sendmDesc ok
- [ ] intringsics kinectv1 are hardcoded
- [ ]
