import importlib.util
from pathlib import Path
import sys
import types
import unittest
from unittest.mock import patch


def load_mediator_module():
    rospy = types.ModuleType("rospy")
    rospy.Subscriber = lambda *args, **kwargs: None
    rospy.loginfo = lambda *args, **kwargs: None
    rospy.Rate = lambda *_args, **_kwargs: types.SimpleNamespace(sleep=lambda: None)
    rospy.resolve_name = lambda topic: topic if topic.startswith("/") else "/robot/" + topic
    object_detection_msgs = types.ModuleType("object_detection_msgs")
    object_detection_msgs_msg = types.ModuleType("object_detection_msgs.msg")

    class CameraFrame:
        def __init__(self, camera_topic, image):
            self.camera_topic = camera_topic
            self.image = image

    object_detection_msgs_msg.CameraFrame = CameraFrame
    robotnik_msgs = types.ModuleType("robotnik_msgs")
    robotnik_msgs_msg = types.ModuleType("robotnik_msgs.msg")

    class State:
        READY_STATE = 300

    robotnik_msgs_msg.State = State
    sensor_msgs = types.ModuleType("sensor_msgs")
    sensor_msgs_msg = types.ModuleType("sensor_msgs.msg")
    sensor_msgs_msg.Image = type("Image", (), {})
    std_msgs = types.ModuleType("std_msgs")
    std_msgs_msg = types.ModuleType("std_msgs.msg")
    std_msgs_msg.String = type("String", (), {})
    modules = {
        "rospy": rospy,
        "object_detection_msgs": object_detection_msgs,
        "object_detection_msgs.msg": object_detection_msgs_msg,
        "robotnik_msgs": robotnik_msgs,
        "robotnik_msgs.msg": robotnik_msgs_msg,
        "sensor_msgs": sensor_msgs,
        "sensor_msgs.msg": sensor_msgs_msg,
        "std_msgs": std_msgs,
        "std_msgs.msg": std_msgs_msg,
    }

    script_path = Path(__file__).parents[1] / "scripts" / "detector_camera_mediator_node.py"
    spec = importlib.util.spec_from_file_location("mediator_under_test", script_path)
    module = importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules, modules):
        spec.loader.exec_module(module)
    return module, State


class DetectorRoutingTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.mediator_module, cls.State = load_mediator_module()

    def test_frame_routes_to_fresh_ready_detectors_for_named_camera(self):
        registry = [
            {"name": "front_only", "state_topic": "/front/state", "image_topic": "/front/image"},
            {"name": "top_only", "state_topic": "/top/state", "image_topic": "/top/image"},
        ]
        discovery = self.mediator_module.ActiveModelDiscovery(registry, 5.0)
        ready = types.SimpleNamespace(state=self.State.READY_STATE)

        publishers = {}
        for detector_name in ("front_only", "top_only"):
            publisher = types.SimpleNamespace(published=[])
            publisher.publish = publisher.published.append
            publishers[detector_name] = publisher
        mediator = self.mediator_module.DetectorCameraMediator.__new__(
            self.mediator_module.DetectorCameraMediator
        )
        mediator.model_discovery = discovery
        mediator._image_publishers = publishers
        mediator.camera_topics = {"front": "/robot/front/image_raw", "top": "/robot/top/image_raw"}
        mediator._detector_cameras = {"front_only": ("front",), "top_only": ("top",)}

        with patch.object(self.mediator_module.time, "monotonic", side_effect=[10.0, 10.0, 10.1, 16.0]):
            discovery.update_from_status("front_only", ready)
            discovery.update_from_status("top_only", ready)
            image = types.SimpleNamespace(header=types.SimpleNamespace(stamp=10, frame_id="camera"))
            self.assertTrue(mediator._publish_frame("front", image))
            routed = publishers["front_only"].published[0]
            self.assertEqual(routed.camera_topic, "/robot/front/image_raw")
            self.assertIs(routed.image, image)
            self.assertEqual(publishers["top_only"].published, [])
            self.assertFalse(mediator._publish_frame("front", image))

    def test_non_ready_state_removes_detector(self):
        registry = [{"name": "first", "state_topic": "/first/state", "image_topic": "/first/image"}]
        discovery = self.mediator_module.ActiveModelDiscovery(registry, 5.0)
        discovery.update_from_status("first", types.SimpleNamespace(state=self.State.READY_STATE))
        discovery.update_from_status("first", types.SimpleNamespace(state=0))

        self.assertEqual(discovery.ready_detectors(), ())

    def test_run_forwards_image_without_changing_header(self):
        mediator = self.mediator_module.DetectorCameraMediator.__new__(
            self.mediator_module.DetectorCameraMediator)
        mediator.processing_rate_hz = 1.0
        image = types.SimpleNamespace(header=types.SimpleNamespace(stamp=42, frame_id="shared"))
        published = []
        mediator._next_frame = lambda: ("camera/image_raw", image)
        mediator._publish_frame = lambda camera_topic, frame: published.append((camera_topic, frame))
        with patch.object(self.mediator_module.rospy, "is_shutdown", side_effect=[False, True], create=True), \
                patch.object(self.mediator_module.rospy, "logdebug", create=True):
            mediator.run()
        self.assertEqual(published[0][0], "camera/image_raw")
        self.assertIs(published[0][1], image)
        self.assertEqual(image.header.stamp, 42)
        self.assertEqual(image.header.frame_id, "shared")


if __name__ == "__main__":
    unittest.main()