# stereo_reconstruction_OpenCV_impl #
Stereo 3D reconstruction with OpenCV using sample code to implement.

# Target #
1. implementation of stereo camera reconstruction(multi-camera)
2. implementation of stereo camrea calibration and compensation.

# Theory #
<br>[single_camera_calibration](https://github.com/DC-Cheng/camera_calibration_OpenCV_impl)
<br>[epipolar geometry](https://en.wikipedia.org/wiki/Epipolar_geometry)
<br>![Image0](https://pic.pimg.tw/silverwind1982/1472199756-4228555464_n.png)

# Related openCV API function #

# Pipeline # 
Step0:
> Prepare dataset, compling environment, and theory.

Step1:
> Calibrate camera - to get camera intrinsics and extrinsic parameters
```
double rms = stereoCalibrate(objectPoints, imagePoints[0], imagePoints[1],
  cameraMatrix[0], distCoeffs[0],
  cameraMatrix[1], distCoeffs[1],
  imageSize, R, T, E, F,
  CALIB_FIX_ASPECT_RATIO +
  CALIB_ZERO_TANGENT_DIST +
  CALIB_USE_INTRINSIC_GUESS +
  CALIB_SAME_FOCAL_LENGTH +
  CALIB_RATIONAL_MODEL +
  CALIB_FIX_K3 + CALIB_FIX_K4 + CALIB_FIX_K5,
  TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 100, 1e-5));
cout << "done with RMS error=" << rms << endl;

	// CALIBRATION QUALITY CHECK
	// because the output fundamental matrix implicitly
	// includes all the output information,
	// we can check the quality of calibration using the
	// epipolar geometry constraint: m2^t*F*m1=0
	double err = 0;
	int npoints = 0;
	vector<Vec3f> lines[2];
	for (i = 0; i < nimages; i++)
	{
		int npt = (int)imagePoints[0][i].size();
		Mat imgpt[2];
		for (k = 0; k < 2; k++)
		{
			imgpt[k] = Mat(imagePoints[k][i]);
			undistortPoints(imgpt[k], imgpt[k], cameraMatrix[k], distCoeffs[k], Mat(), cameraMatrix[k]);
			computeCorrespondEpilines(imgpt[k], k + 1, F, lines[k]);
		}
		for (j = 0; j < npt; j++)
		{
			double errij = fabs(imagePoints[0][i][j].x*lines[1][j][0] +
				imagePoints[0][i][j].y*lines[1][j][1] + lines[1][j][2]) +
				fabs(imagePoints[1][i][j].x*lines[0][j][0] +
					imagePoints[1][i][j].y*lines[0][j][1] + lines[0][j][2]);
			err += errij;
		}
		npoints += npt;
	}
	cout << "average epipolar err = " << err / npoints << endl;

```
Rectify Image - Align image pixels on epipolar lines to aid in disparity generation
```
stereoRectify(cameraMatrix[0], distCoeffs[0],
cameraMatrix[1], distCoeffs[1],
imageSize, R, T, R1, R2, P1, P2, Q,
CALIB_ZERO_DISPARITY, 1, imageSize, &validRoi[0], &validRoi[1]);
```
```
F = findFundamentalMat(Mat(allimgpt[0]), Mat(allimgpt[1]), FM_8POINT, 0, 0);
```
```
Mat H1, H2;
stereoRectifyUncalibrated(Mat(allimgpt[0]), Mat(allimgpt[1]), F, imageSize, H1, H2, 3);

R1 = cameraMatrix[0].inv()*H1*cameraMatrix[0];
R2 = cameraMatrix[1].inv()*H2*cameraMatrix[1];
P1 = cameraMatrix[0];
P2 = cameraMatrix[1];
```
then we can get:
`output_intrinsic_parameter.yml`, `output_extrinsic_parameter.yml`
<br>![rectified_result](https://github.com/DC-Cheng/stereo_reconstruction_OpenCV_impl/blob/master/rectified_result.png?raw=true)

Step2:
> Generate depth maps
```

numberOfDisparities = numberOfDisparities > 0 ? numberOfDisparities : ((img_size.width / 8) + 15) & -16;

bm->setROI1(roi1);
bm->setROI2(roi2);
bm->setPreFilterCap(31);
bm->setBlockSize(SADWindowSize > 0 ? SADWindowSize : 9);
bm->setMinDisparity(0);
bm->setNumDisparities(numberOfDisparities);
bm->setTextureThreshold(10);
bm->setUniquenessRatio(15);
bm->setSpeckleWindowSize(100);
bm->setSpeckleRange(32);
bm->setDisp12MaxDiff(1);

sgbm->setPreFilterCap(63);
int sgbmWinSize = SADWindowSize > 0 ? SADWindowSize : 3;
sgbm->setBlockSize(sgbmWinSize);

int cn = img1.channels();

sgbm->setP1(8 * cn*sgbmWinSize*sgbmWinSize);
sgbm->setP2(32 * cn*sgbmWinSize*sgbmWinSize);
sgbm->setMinDisparity(0);
sgbm->setNumDisparities(numberOfDisparities);
sgbm->setUniquenessRatio(10);
sgbm->setSpeckleWindowSize(100);
sgbm->setSpeckleRange(32);
sgbm->setDisp12MaxDiff(1);
if (alg == STEREO_HH)
	sgbm->setMode(StereoSGBM::MODE_HH);
else if (alg == STEREO_SGBM)
	sgbm->setMode(StereoSGBM::MODE_SGBM);
else if (alg == STEREO_3WAY)
	sgbm->setMode(StereoSGBM::MODE_SGBM_3WAY);
```
<br>![stereo_match_result.png](https://github.com/DC-Cheng/stereo_reconstruction_OpenCV_impl/blob/master/stereo_match_result.png?raw=true)

Step3:
> Perform 3D reconstruction- Project 2d pixels into its real world 3D coordinates
<br>![result_3d.png](https://github.com/DC-Cheng/stereo_reconstruction_OpenCV_impl/blob/master/result_3d.png?raw=true)

# Results(Meta) #
As you can see the disparity images of 2d map and 3d results are significantly with wrong 3d data output.
The dataSet is prepared for stereo_calibration sample code without hardware spec.
Therefore, it is important to check the `hardware parameter` in order to `avoid from getting wrong 3d data value`.
At this point, I may need a more complete dataSet to get a good one, not a worse one like this.

# Reference #
[Omar Padierna](https://medium.com/@omar.ps16)
<br> [part1](https://becominghuman.ai/stereo-3d-reconstruction-with-opencv-using-an-iphone-camera-part-i-c013907d1ab5)
<br> [part2](https://medium.com/@omar.ps16/stereo-3d-reconstruction-with-opencv-using-an-iphone-camera-part-ii-77754b58bfe0)
<br> [part3](https://medium.com/@omar.ps16/stereo-3d-reconstruction-with-opencv-using-an-iphone-camera-part-iii-95460d3eddf0)
<br> [github-amroamroamro](http://amroamroamro.github.io/mexopencv/opencv/stereo_calibration_demo.html)
<br> [github-abhileshborode](https://github.com/abhileshborode/Stereo-depth-reconstruction)
<br> [OpenCV-stereo_calibrate-example1](https://github.com/opencv/opencv/blob/3.2.0/samples/cpp/stereo_calib.cpp)
<br> [OpenCV-stereo_calibrate-example2](https://ithelp.ithome.com.tw/articles/10223959)
<br> [OpenCV-stereo_depthMatch-sample-code](https://docs.opencv.org/master/dd/d53/tutorial_py_depthmap.html)

# Hint #
其中argv命令引數的部分，這個範例使用了`cv::CommandLineParser`
你可以修改input || 裡面的設定檔檔名
```
	cv::CommandLineParser parser(argc, argv, "{w|9|}{h|6|}{s|50.0|}{nr||}{help|1|}{@input|stereo_calib.xml|}");
	if (parser.has("help"))
		return print_help();
	showRectified = !parser.has("nr");
	imagelistfn = samples::findFile(parser.get<string>("@input"));
	boardSize.width = parser.get<int>("w");
	boardSize.height = parser.get<int>("h");
	float squareSize = parser.get<float>("s");
  ```

如何修改"argv命令引數" in visual studio
<br>[VS設定命列參數列-ref1](https://edisonx.pixnet.net/blog/post/57060736)
<br>[VS設定命列參數列-ref2](https://www.itread01.com/p/879116.html)
<br>[VS設定命列參數列-ref3](https://social.msdn.microsoft.com/Forums/en-US/20865ea1-ff94-41a7-b668-a7f24154f3b4/argc-and-argv-inputs?forum=vcmfcatl)
<br>[CommandLineParser使用](https://www.itread01.com/content/1541983112.html)
  
