# What is it?
A ROS robot_localization demo running completely on Android.

# What does it use?
* Rosjava.
* A cross-compiled native code version of the robot_localization package.
* An IMU data publisher node.

# What do I need to build it?
The following must be installed on your system:
* ROS
* Rosjava
* Android SDK
* Java SDK 1.6 or greater
* GCC

# How do you build it?
Standing in the topmost folder run the following command:
```
catkin_make
```
If everything goes well you should have the android package located in the following folder:
```
~/android_roloc/src/androidpkg1/androidp1/build/outputs/apk/androidp1-debug.apk
```

# How do I test it?
Standing in the package output folder and with your Android device plugged-in run the following:
```
adb install -r androidp1-debug.apk
```
You should now have the demo app "Androidp1" installed on your device. As soon as you start it you will see a screen to choose the ROS master. Tap on "Show advanced options" and select the "New Public Master". After this the master will start and the nodes will load.
To connect to the master running on the device from your PC you must be on the same network. Also you will need to set the environment variable to tell ROS where the master is:
```
export ROS_MASTER_URI=http://192.168.0.1:11311

```
Replace the IP by the one assigned to your Android device.
You can now interact with the ROS system running on the device.
List the nodes:
```
rosnode list
```
List the topics:
```
rostopic list
```
Listen to the published localization result:
```
rostopic echo /odometry/filtered
```
Note: on this example the localization data is virtually unusuable since we are integrating an acceleration and the error keeps accumulating, making the position estimation to diverge.
