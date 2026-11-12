# detector_camera_mediator

ROS 1 node that subscribes to configured `sensor_msgs/Image` camera topics,
queues frames per camera, and processes them round-robin. Old queued frames are
dropped when a camera queue is full. Each frame is published to every configured
detector whose state is READY; each detector has its own image input topic.
Detector inputs use `object_detection_msgs/CameraFrame`: the resolved camera
topic identifies the source independently of the unchanged image header.

![Diagram of the mediator](./docs/detector_camera_mediator.png)

## Configure

Edit `config/detector_camera_mediator.yaml` and list camera topics in order:

```yaml
camera_topics:
  - /camera/image_raw
```

Set `frame_queue_size`, `model_status_timeout`, and `detector_registry` as
needed. Each registry entry contains a unique detector `name`, its
`robotnik_msgs/State` `state_topic`, and dedicated `image_topic`. A detector
receives frames while it reports `READY_STATE` (300); READY expires if no state
update arrives before `model_status_timeout`.

## Build and run

From the root of a ROS 1 catkin workspace containing this package:

```bash
catkin_make
source devel/setup.bash
roslaunch detector_camera_mediator detector_camera_mediator.launch
```

The node does not start or stop models.

Model memory is managed by the `object_detection` detector controllers, through
their existing `activate_detector` services (directly or through the detector
manager). Activation loads the configured model in a child process;
deactivation stops and joins that process while leaving the ROS controller
available for later reactivation. A detector reports READY only after loading
and subscribing to its image input. The mediator's READY tracking remains an
image-routing decision, not a model lifecycle trigger.
