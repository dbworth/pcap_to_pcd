
# PCAP to PCD 

A tool to convert a .pcap file from a Velodyne VLP-16 LiDAR sensor <img src="https://raw.githubusercontent.com/dbworth/pcap_to_pcd/master/Screenshots/Velodyne_VLP-16_Viewer_1.png" width="300" align="right">  
to multiple .pcd PointCloud files.  

This stand-alone VLP Grabber can save you time from  
compiling PCL (PointCloud Library) from source.

**Author:** David Butterworth, 2019.  
using code from PCL and Tsukasa Sugiura.  


### Installation:

Code was tested on: 
 - Ubuntu 16.04 with PCL 1.7.2  
 - Ubuntu 18.04 with PCL 1.8.1  

Dependencies:  
`sudo apt install libpcl-dev libpcap-dev pcl-tools`

Compile:  
```
mkdir build
cd build/
cmake ..
make
```

Sample data:  
http://midas3.kitware.com/midas/community/29/


## Usage:

Playback the data in PCL Visualizer:  
`./main -pcap /path/to/data.pcap`  
Press 'q' key to exit.  

Convert the data to .pcd files:  
`./main -pcap /path/to/data.pcap -saveframes 1`  
This saves each scan as a separate file  
e.g.  
`cloud00000.pcd`  
`cloud00001.pcd`  
`cloud00002.pcd`  
etc.  

To view one of the scans:  
`pcl_viewer cloud00000.pcd`

