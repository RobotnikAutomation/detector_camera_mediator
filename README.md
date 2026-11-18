# detector_camera_mediator

ROS node that routes camera images to READY detectors. It queues frames per
camera, drops the oldest when a queue is full, and dispatches round-robin at a
configurable total rate (10 Hz by default, at most one frame per tick).
Each detector receives an `object_detection_msgs/CameraFrame` with the resolved
source topic in `camera_topic` and the original image in `image`. Status is
published as JSON every five seconds on `/detector_camera_mediator/status`.

![Diagram of the mediator](./docs/detector_camera_mediator.png)

## Related packages

- [object_detection](https://github.com/RobotnikAutomation/object_detection)

## Configure

Edit `config/detector_camera_mediator.yaml` to set the named camera inputs,
queue capacity, and detector routing registry. The current configuration is:

```yaml
camera_topics:
  front: /robot/front_rgbd_camera/color/image_raw
  top: /robot/top_ptz_camera/color/image_raw
  thermal: /robot/top_ptz_camera/thermal/image_raw

# Maximum number of pending frames to retain per camera; oldest frames are dropped first.
frame_queue_size: 2

# Maximum work dispatches per second across all cameras.
processing_rate_hz: 10.0

# A detector receives frames on its image_topic only while its state is READY.
detector_registry:
  - name: person_detector
    state_topic: /robot/detectors/person_detector/state
    image_topic: /robot/detectors/person_detector/image
    camera_topics: [top]
  - name: broken_fence_detector
    state_topic: /robot/detectors/broken_fence_detector/state
    image_topic: /robot/detectors/broken_fence_detector/image
    camera_topics: [front, thermal]
  - name: vehicle_detector
    state_topic: /robot/detectors/vehicle_detector/state
    image_topic: /robot/detectors/vehicle_detector/image
    camera_topics: [front]
  - name: multiclass_detector
    state_topic: /robot/detectors/multiclass_detector/state
    image_topic: /robot/detectors/multiclass_detector/image_raw
    camera_topics: [top]
  - name: fire_detector
    state_topic: /robot/detectors/fire_detector/state
    image_topic: /robot/detectors/fire_detector/image_raw
    camera_topics: [thermal]

# READY is considered stale if no state update arrives within this interval.
model_status_timeout: 5.0
```

`camera_topics` maps camera names to ROS image topics. Each detector's `camera_topics` list selects
which named cameras it receives. `frame_queue_size` is the maximum number of
pending images per camera, with the oldest dropped when full.
`processing_rate_hz` sets the dispatch rate across all cameras, not per camera.
Each registry entry must have a unique detector `name`, its `robotnik_msgs/State`
`state_topic`, dedicated `image_topic`, and valid camera names. A detector
receives frames while it reports `READY_STATE` (300); READY expires if no state
update arrives before `model_status_timeout`.

Detectors receive `CameraFrame` messages on one input topic and publish
`object_detection_msgs/CameraDetections`
on `camera_detections` for the annotator. The wrapper carries the source camera
topic separately from the unchanged image header. The standard `bounding_boxes`
output remains a `vision_msgs/Detection2DArray` for other consumers.

`inspection_bringup` uses
`inspection_bringup/config/ai_perception/detector_camera_mediator.yaml` instead
of this package's config. Keep both files aligned.

## Build and run

From a ROS catkin workspace:

```bash
catkin_make
source devel/setup.bash
roslaunch detector_camera_mediator detector_camera_mediator.launch
```

The mediator only routes images; it does not manage model lifecycle. The
`object_detection` controllers manage models through their `activate_detector`
services and report READY after loading and subscribing to the image input.
