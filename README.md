### Simple quadrotor controller implemented in simulator from Udacity [FCND-Controls-CPP project](https://github.com/udacity/FCND-Controls-CPP)
### Control structure with Euler angles
![control structire](https://github.com/ViktorAnchutin/FCND-Simple-Quadrotor-Controller/blob/master/img/control.png?raw=true)

### Altitude controller


FCND-Simple-Quadrotor-Controller/prj/src/QuadControl.cpp:
```C++
float QuadControl::AltitudeControl(float posZCmd, float posZ)
```

Functiomal scheme:

![](https://github.com/ViktorAnchutin/FCND-Simple-Quadrotor-Controller/blob/master/img/altitude.png?raw=true)



### Position controller

FCND-Simple-Quadrotor-Controller/prj/src/QuadControl.cpp:
```C++
V3F QuadControl::PositionControl(V3F posCmd, V3F pos, V3F vel)
```

Functional scheme:

![](https://github.com/ViktorAnchutin/FCND-Simple-Quadrotor-Controller/blob/master/img/position.png?raw=true)


### Roll-Pitch controller

FCND-Simple-Quadrotor-Controller/prj/src/QuadControl.cpp:
```C++
V3F QuadControl::RollPitchControl(V3F des_angles_I_frame, Quaternion<float> attitude)
```

Functional scheme:

![](https://github.com/ViktorAnchutin/FCND-Simple-Quadrotor-Controller/blob/master/img/rollpitch.PNG?raw=true)



