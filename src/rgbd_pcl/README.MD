


# FC 是Front_Cam的缩写、SC是Side_Cam的缩写。分别对应前置和机械臂摄像头



奥比中光 rgb相机标定指令：
```
rosrun camera_calibration cameracalibrator.py --size 11x8 --square 0.020 image:=/camera/color/image_raw camera:=/camera/color
```

首先，你需要在奥比中光的launch文件中修改<camera_name>(通常在launch文件的开头)。
比如现在使用的是gemini_336l,则改为：gemini_336l

标定完成后文件需要重命名为：```<camera_name>_color.yaml```或者```<camera_name>_ir.yaml```，取决于你标定的是哪个镜头

注意，这里的camera_name和上一步一样。
这样奥比中光的驱动能够直接加载到yaml文件。

下一步，打开标定文件，将其中的camera_name替换成```<camera_name>_color```或者```<camera_name>_ir```。三个名字都匹配上就不会报错。

[//]: # (或者，你可以手动在launch文件中指定)


如果camera_name出了问题,通常会报错：
```
[CameraInfoManager::loadCalibrationFile]: 
[gemini_336l_color] does not match name 
gemini_336l in file /home/hiling/.ros/camera_info
/gemini_336l_color.yaml
```


