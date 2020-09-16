# KALMAN FILTER BOUNDING BOXES

Kalman example to track the mobile phone using YOLO-V4 tiny detector.

---

## Dependencies

The detector used is YOLOV4, using AlexeyAB implementation: https://github.com/AlexeyAB/darknet

```
git clone https://github.com/AlexeyAB/darknet.git; cd darknet;
mkdir build_release; cd build_release; 
cmake .. ; make -j; sudo make install
```

It's necesary change [this line](https://github.com/mgrova/bear/blob/d40c50c22fb53f659c6165cc751c5a69b9d560f6/examples/kf_bboxes/CMakeLists.txt#L26) of CMakeLists.txt to your own path.

## Downloads

* YOLO V4 TINY
  * Weights: https://github.com/AlexeyAB/darknet/releases/download/darknet_yolo_v4_pre/yolov4-tiny.weights
  * Cfg: https://raw.githubusercontent.com/AlexeyAB/darknet/master/cfg/yolov4.cfg