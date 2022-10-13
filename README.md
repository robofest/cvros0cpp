# `cvros0cpp` package to introduce OpenCV in ROS with C++
There are 4 independent nodes. Use the following launch files to test them.
* Simple image view, C style<br>
`$ roslaunch cvros0cpp view_C.launch`
* Simple image view, Object Oriented <br>
`$ roslaunch cvros0cpp view_OO.launch`
* OO style, to draw a red circle on a color image<br>
`$ roslaunch cvros0cpp draw1Circle.launch`
* Calculate White Percent of B/W image, OO style<br>
`$ roslaunch cvros0cpp cntWhitePixels.launch`