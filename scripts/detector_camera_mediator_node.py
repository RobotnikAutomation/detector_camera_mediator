#!/usr/bin/env python3

from collections import deque
import json
import threading
import time

import rospy
from object_detection_msgs.msg import CameraFrame
from robotnik_msgs.msg import State
from sensor_msgs.msg import Image
from std_msgs.msg import String


class ActiveModelDiscovery:
    """Track READY detector entries and expire status that stops updating."""

    def __init__(self, detector_registry, status_timeout):
        self.detector_registry = detector_registry
        self.status_timeout = status_timeout
        self._ready_at = {}
        self._lock = threading.Lock()
        self._subscribers = [
            rospy.Subscriber(
                detector["state_topic"],
                State,
                self._make_status_callback(detector["name"]),
                queue_size=1,
            )
            for detector in self.detector_registry
        ]

        if self.detector_registry:
            rospy.loginfo("Tracking READY state for %d detectors", len(self.detector_registry))
        else:
            rospy.loginfo("No detectors configured")

    def _make_status_callback(self, detector_name):
        """Return a callback that updates activity for one detector."""
        def update(status_message):
            self.update_from_status(detector_name, status_message)

        return update

    def ready_detectors(self):
        """Return detector names with fresh READY status."""
        now = time.monotonic()
        with self._lock:
            expired = [
                name for name, ready_at in self._ready_at.items()
                if now - ready_at > self.status_timeout
            ]
            for name in expired:
                del self._ready_at[name]
            return tuple(self._ready_at)

    def update_from_status(self, detector_name, status_message):
        """Mark a detector active only while its state is READY_STATE."""
        with self._lock:
            if status_message.state == State.READY_STATE:
                self._ready_at[detector_name] = time.monotonic()
            else:
                self._ready_at.pop(detector_name, None)


class DetectorCameraMediator:
    """Subscribe to configured camera topics and queue incoming frames."""

    def __init__(self):
        """Load camera settings and create one subscriber per topic."""
        camera_topics = rospy.get_param("~camera_topics", [])
        self.camera_topics = list(dict.fromkeys(camera_topics))
        self.queue_size = int(rospy.get_param("~frame_queue_size", 2))
        self.processing_rate_hz = float(rospy.get_param("~processing_rate_hz", 10.0))
        detector_registry = rospy.get_param("~detector_registry", [])
        self.status_timeout = float(rospy.get_param("~model_status_timeout", 5.0))

        if self.queue_size < 1:
            raise ValueError("~frame_queue_size must be at least 1")
        if self.processing_rate_hz <= 0:
            raise ValueError("~processing_rate_hz must be greater than 0")
        if self.status_timeout <= 0:
            raise ValueError("~model_status_timeout must be greater than 0")
        if not isinstance(detector_registry, list):
            raise ValueError("~detector_registry must be a list")
        required_fields = ("name", "state_topic", "image_topic")
        for detector in detector_registry:
            if not isinstance(detector, dict) or any(
                not isinstance(detector.get(field), str) or not detector[field]
                for field in required_fields
            ):
                raise ValueError(
                    "~detector_registry entries need name, state_topic, and image_topic strings"
                )
        detector_names = [detector["name"] for detector in detector_registry]
        if len(set(detector_names)) != len(detector_names):
            raise ValueError("~detector_registry names must be unique")

        self._queues = {
            topic: deque() for topic in self.camera_topics
        }
        self.model_discovery = ActiveModelDiscovery(
            detector_registry, self.status_timeout
        )
        self._image_publishers = {
            detector["name"]: rospy.Publisher(detector["image_topic"], CameraFrame, queue_size=1)
            for detector in detector_registry
        }
        self._status_publisher = rospy.Publisher("~status", String, queue_size=1)
        self._status_timer = rospy.Timer(
            rospy.Duration(5.0), self._publish_status
        )
        self._condition = threading.Condition()
        self._next_camera_index = 0
        self._subscribers = [
            rospy.Subscriber(
                topic,
                Image,
                self._make_callback(topic),
                queue_size=1,
            )
            for topic in self.camera_topics
        ]

        if not self.camera_topics:
            rospy.logwarn("No camera topics configured")

    def _make_callback(self, camera_topic):
        """Return a callback that enqueues frames for one camera."""
        def enqueue_frame(frame):
            with self._condition:
                camera_queue = self._queues[camera_topic]
                if len(camera_queue) >= self.queue_size:
                    camera_queue.popleft()
                    rospy.logwarn_throttle(
                        5.0,
                        "Dropping oldest queued frame from %s", camera_topic
                    )
                camera_queue.append(frame)
                self._condition.notify()

        return enqueue_frame

    def _next_frame(self):
        """Return the next available frame in round-robin camera order, if any."""
        with self._condition:
            topic_count = len(self.camera_topics)
            for offset in range(topic_count):
                index = (self._next_camera_index + offset) % topic_count
                topic = self.camera_topics[index]
                if self._queues[topic]:
                    self._next_camera_index = (index + 1) % topic_count
                    return topic, self._queues[topic].popleft()

        return None

    def _publish_frame(self, camera_topic, frame):
        """Forward an image with its source topic to each fresh READY detector."""
        ready_detectors = self.model_discovery.ready_detectors()
        if not ready_detectors:
            return False
        routed_frame = CameraFrame(
            camera_topic=rospy.resolve_name(camera_topic), image=frame
        )
        for detector_name in ready_detectors:
            self._image_publishers[detector_name].publish(routed_frame)
        return bool(ready_detectors)

    def _publish_status(self, _event=None):
        """Publish the configured work rate and currently READY detectors."""
        active_detectors = self.model_discovery.ready_detectors()
        status = {
            "processing_rate_hz": self.processing_rate_hz,
            "active_detector_count": len(active_detectors),
            "active_detectors": active_detectors,
        }
        self._status_publisher.publish(String(data=json.dumps(status)))

    def run(self):
        """Dispatch at most one queued frame per processing-rate tick."""
        rate = rospy.Rate(self.processing_rate_hz)
        while not rospy.is_shutdown():
            next_frame = self._next_frame()
            if next_frame is not None:
                topic, frame = next_frame
                self._publish_frame(topic, frame)
                rospy.logdebug("Processed frame from %s", topic)
            rate.sleep()


def main():
    """Initialize ROS and run the camera frame queue."""
    rospy.init_node("detector_camera_mediator")
    mediator = DetectorCameraMediator()
    mediator.run()


if __name__ == "__main__":
    main()