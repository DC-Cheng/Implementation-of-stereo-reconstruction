# stereo_reconstruction_OpenCV_impl #
Stereo 3D reconstruction with OpenCV using an iPhone camera.                <br>

# Target #
1. implementation of stereo camera reconstruction(multi-camera)
2. implementation of stereo camrea calibration and compensation.

# Theory #
[epipolar geometry](https://en.wikipedia.org/wiki/Epipolar_geometry)
<br>![Image0](https://pic.pimg.tw/silverwind1982/1472199756-4228555464_n.png)

# Related openCV API function #

# Pipeline # 
> prepare dataset, compling environment, and theory.
<br> Calibrate camera - camera intrinsics and extrinsic parameters
<br> Rectify Image - Align image pixels on epipolar lines to aid in disparity generation
<br> Generate depth maps
<br> Perform 3D reconstruction- Project 2d pixels into its real world 3D coordinates
<br> //ref: [github-abhileshborode](https://github.com/abhileshborode/Stereo-depth-reconstruction)

# Reference #
[OpenCV sample code](https://docs.opencv.org/master/dd/d53/tutorial_py_depthmap.html)
<br>
[Omar Padierna: ](https://medium.com/@omar.ps16)
[part1](https://becominghuman.ai/stereo-3d-reconstruction-with-opencv-using-an-iphone-camera-part-i-c013907d1ab5)
[part2](https://medium.com/@omar.ps16/stereo-3d-reconstruction-with-opencv-using-an-iphone-camera-part-ii-77754b58bfe0)
[part3](https://medium.com/@omar.ps16/stereo-3d-reconstruction-with-opencv-using-an-iphone-camera-part-iii-95460d3eddf0)
