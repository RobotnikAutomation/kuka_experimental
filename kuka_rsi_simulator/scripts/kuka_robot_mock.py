#!/usr/bin/env python3

import argparse
import socket
import time
import xml.etree.ElementTree as ET

import rospy
from std_msgs.msg import String


def _to_float(value, default=0.0):
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def _to_int(value, default=0):
    try:
        return int(value)
    except (TypeError, ValueError):
        return default


class KukaRobotMock(object):
    def __init__(self, host, port):
        self.host = host
        self.port = int(port)

        self.node_name = rospy.get_name()
        self.cycle_time = float(rospy.get_param("~cycle_time", 0.004))
        self.socket_timeout = float(rospy.get_param("~socket_timeout", 1.0))
        self.log_raw_xml = bool(rospy.get_param("~log_raw_xml", False))

        self.timeout_count = 0
        self.ipoc = 1

        self.actual_joint_pos = self._get_pose_param(
            "~initial_joint_pose", [0.0, -90.0, 90.0, 0.0, 90.0, 0.0]
        )
        self.setpoint_joint_pos = list(self.actual_joint_pos)
        self.actual_cart_pose = self._get_pose_param(
            "~initial_cart_pose", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        )
        self.setpoint_cart_pose = list(self.actual_cart_pose)

        self.rsi_state_pub = rospy.Publisher(self.node_name + "/rsi/state", String, queue_size=1)
        self.rsi_command_pub = rospy.Publisher(self.node_name + "/rsi/command", String, queue_size=1)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(self.socket_timeout)
        rospy.loginfo("%s: UDP socket ready (target=%s:%d)", self.node_name, self.host, self.port)

        rospy.on_shutdown(self._on_shutdown)

    def _on_shutdown(self):
        try:
            self.sock.close()
        except Exception:
            pass

    def _get_pose_param(self, name, default_value):
        value = rospy.get_param(name, default_value)
        if not isinstance(value, list) or len(value) != 6:
            rospy.logwarn("%s: invalid %s, using default %s", self.node_name, name, default_value)
            return list(default_value)
        return [_to_float(v) for v in value]

    def _build_robot_state_xml(self):
        root = ET.Element("Rob", {"TYPE": "KUKA"})

        ET.SubElement(
            root,
            "RIst",
            {
                "X": str(self.actual_cart_pose[0]),
                "Y": str(self.actual_cart_pose[1]),
                "Z": str(self.actual_cart_pose[2]),
                "A": str(self.actual_cart_pose[3]),
                "B": str(self.actual_cart_pose[4]),
                "C": str(self.actual_cart_pose[5]),
            },
        )
        ET.SubElement(
            root,
            "RSol",
            {
                "X": str(self.setpoint_cart_pose[0]),
                "Y": str(self.setpoint_cart_pose[1]),
                "Z": str(self.setpoint_cart_pose[2]),
                "A": str(self.setpoint_cart_pose[3]),
                "B": str(self.setpoint_cart_pose[4]),
                "C": str(self.setpoint_cart_pose[5]),
            },
        )
        ET.SubElement(
            root,
            "AIPos",
            {
                "A1": str(self.actual_joint_pos[0]),
                "A2": str(self.actual_joint_pos[1]),
                "A3": str(self.actual_joint_pos[2]),
                "A4": str(self.actual_joint_pos[3]),
                "A5": str(self.actual_joint_pos[4]),
                "A6": str(self.actual_joint_pos[5]),
            },
        )
        ET.SubElement(
            root,
            "ASPos",
            {
                "A1": str(self.setpoint_joint_pos[0]),
                "A2": str(self.setpoint_joint_pos[1]),
                "A3": str(self.setpoint_joint_pos[2]),
                "A4": str(self.setpoint_joint_pos[3]),
                "A5": str(self.setpoint_joint_pos[4]),
                "A6": str(self.setpoint_joint_pos[5]),
            },
        )
        ET.SubElement(root, "Delay", {"D": str(self.timeout_count)})
        ET.SubElement(root, "IPOC").text = str(self.ipoc)

        return ET.tostring(root, encoding="utf-8")

    def _parse_command_xml(self, xml_bytes):
        try:
            root = ET.fromstring(xml_bytes)
        except ET.ParseError as ex:
            rospy.logwarn_throttle(1.0, "%s: invalid command XML: %s", self.node_name, ex)
            return None, None, None

        cart_corr = [0.0] * 6
        axis_corr = [0.0] * 6

        cart_el = root.find("RKorr")
        if cart_el is None:
            cart_el = root.find("RK")
        if cart_el is not None:
            cart_corr = [
                _to_float(cart_el.attrib.get("X", 0.0)),
                _to_float(cart_el.attrib.get("Y", 0.0)),
                _to_float(cart_el.attrib.get("Z", 0.0)),
                _to_float(cart_el.attrib.get("A", 0.0)),
                _to_float(cart_el.attrib.get("B", 0.0)),
                _to_float(cart_el.attrib.get("C", 0.0)),
            ]

        axis_el = root.find("AK")
        if axis_el is not None:
            axis_corr = [
                _to_float(axis_el.attrib.get("A1", 0.0)),
                _to_float(axis_el.attrib.get("A2", 0.0)),
                _to_float(axis_el.attrib.get("A3", 0.0)),
                _to_float(axis_el.attrib.get("A4", 0.0)),
                _to_float(axis_el.attrib.get("A5", 0.0)),
                _to_float(axis_el.attrib.get("A6", 0.0)),
            ]

        ipoc = None
        ipoc_el = root.find("IPOC")
        if ipoc_el is not None and ipoc_el.text:
            ipoc = _to_int(ipoc_el.text, None)

        return cart_corr, axis_corr, ipoc

    def _apply_corrections(self, cart_corr, axis_corr):
        for i in range(6):
            self.actual_cart_pose[i] += cart_corr[i]
            self.setpoint_cart_pose[i] = self.actual_cart_pose[i]

            self.actual_joint_pos[i] += axis_corr[i]
            self.setpoint_joint_pos[i] = self.actual_joint_pos[i]

    def run(self):
        rospy.loginfo(
            "%s: mock running (cycle_time=%.4fs, socket_timeout=%.2fs)",
            self.node_name,
            self.cycle_time,
            self.socket_timeout,
        )

        while not rospy.is_shutdown():
            cycle_start = time.monotonic()
            state_xml = self._build_robot_state_xml()
            state_text = state_xml.decode("utf-8")
            self.rsi_state_pub.publish(state_text)

            if self.log_raw_xml:
                rospy.loginfo_throttle(1.0, "%s: state -> %s", self.node_name, state_text)

            try:
                self.sock.sendto(state_xml, (self.host, self.port))
                recv_msg, _ = self.sock.recvfrom(4096)
                cmd_text = recv_msg.decode("utf-8", errors="replace")
                self.rsi_command_pub.publish(cmd_text)

                if self.log_raw_xml:
                    rospy.loginfo_throttle(1.0, "%s: command <- %s", self.node_name, cmd_text)

                cart_corr, axis_corr, ipoc_rx = self._parse_command_xml(recv_msg)
                if cart_corr is not None and axis_corr is not None:
                    self._apply_corrections(cart_corr, axis_corr)
                    self.timeout_count = 0
                else:
                    self.timeout_count += 1

                if ipoc_rx is None:
                    self.ipoc += 1
                else:
                    self.ipoc = ipoc_rx + 1

            except socket.timeout:
                self.timeout_count += 1
                self.ipoc += 1
                rospy.logwarn_throttle(2.0, "%s: socket timeout waiting RSI command", self.node_name)
            except OSError as ex:
                rospy.logerr_throttle(1.0, "%s: socket error: %s", self.node_name, ex)
                self.timeout_count += 1
                self.ipoc += 1

            elapsed = time.monotonic() - cycle_start
            sleep_time = self.cycle_time - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)


def _parse_cli_args():
    parser = argparse.ArgumentParser(description="KUKA RSI UDP robot mock")
    parser.add_argument("rsi_hw_iface_ip", help="IP address of the RSI hardware interface")
    parser.add_argument("rsi_hw_iface_port", help="Port of the RSI hardware interface")
    args, _ = parser.parse_known_args()
    return args


def main():
    args = _parse_cli_args()
    rospy.init_node("kuka_robot_mock", anonymous=False)
    mock = KukaRobotMock(args.rsi_hw_iface_ip, args.rsi_hw_iface_port)
    mock.run()


if __name__ == "__main__":
    main()
