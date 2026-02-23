#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time

import rospy
from std_msgs.msg import Bool

from kuka_rsi_cartesian_hw_interface.srv import set_A1_A6


class A1A6Alternator(object):
    """Alternates A6 between two values by calling setKukaA1A6."""

    def __init__(self):
        self.a1_value = rospy.get_param("~a1_value", 0.0)
        self.a6_min = rospy.get_param("~a6_min", 10.0)
        self.a6_max = rospy.get_param("~a6_max", 100.0)
        self.total_duration = rospy.get_param("~duration", 60.0)
        self.pause_s = rospy.get_param("~pause_between_moves", 10.0)
        self.wait_timeout_s = rospy.get_param("~wait_timeout", 30.0)
        self.moving_topic = rospy.get_param("~moving_topic", "/kuka_robot/kuka_moving")
        self.service_name = rospy.get_param("~a1a6_service", "/kuka_robot/setKukaA1A6")

        self.robot_moving = None
        rospy.Subscriber(self.moving_topic, Bool, self._moving_cb, queue_size=1)

        rospy.loginfo(
            "moving_only_a6 params: a1=%.2f a6_min=%.2f a6_max=%.2f duration=%.1fs pause=%.1fs "
            "wait_timeout=%.1fs service=%s moving_topic=%s",
            self.a1_value,
            self.a6_min,
            self.a6_max,
            self.total_duration,
            self.pause_s,
            self.wait_timeout_s,
            self.service_name,
            self.moving_topic,
        )

        rospy.loginfo("Waiting for service: %s", self.service_name)
        rospy.wait_for_service(self.service_name)
        self.set_kuka_a1_a6 = rospy.ServiceProxy(self.service_name, set_A1_A6)
        rospy.loginfo("Service available: %s", self.service_name)

    def _moving_cb(self, msg):
        self.robot_moving = msg.data

    def _wait_first_state(self, timeout_s=2.0):
        start = rospy.Time.now().to_sec()
        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown() and self.robot_moving is None:
            if timeout_s > 0 and (rospy.Time.now().to_sec() - start) >= timeout_s:
                rospy.logwarn(
                    "No message received on %s yet. Assuming robot is stopped.",
                    self.moving_topic,
                )
                self.robot_moving = False
                return
            rate.sleep()

    def _wait_until_stopped(self):
        start = rospy.Time.now().to_sec()
        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            if self.robot_moving is False:
                return True
            if self.wait_timeout_s > 0 and (rospy.Time.now().to_sec() - start) >= self.wait_timeout_s:
                rospy.logwarn("Timeout waiting robot stop on topic: %s", self.moving_topic)
                return False
            rate.sleep()
        return False

    def _call_set_a1_a6(self, a1, a6):
        rospy.loginfo("Calling %s with A1=%.2f A6=%.2f", self.service_name, a1, a6)
        try:
            resp = self.set_kuka_a1_a6(a1, a6)
            if not resp.ret:
                rospy.logwarn("Service returned ret=False for A1=%.2f A6=%.2f", a1, a6)
            return resp.ret
        except rospy.ServiceException as exc:
            rospy.logerr("Service call failed: %s", exc)
            return False

    def run(self):
        self._wait_first_state()

        deadline = time.time() + self.total_duration
        targets = [self.a6_min, self.a6_max]
        index = 0

        while not rospy.is_shutdown() and time.time() < deadline:
            target_a6 = targets[index % len(targets)]

            if not self._call_set_a1_a6(self.a1_value, target_a6):
                break
            if not self._wait_until_stopped():
                break

            index += 1
            if self.pause_s > 0.0:
                rospy.sleep(self.pause_s)

        rospy.loginfo("moving_only_a6 finished.")


def main():
    rospy.init_node("moving_only_a6", anonymous=False)
    A1A6Alternator().run()


if __name__ == "__main__":
    main()
