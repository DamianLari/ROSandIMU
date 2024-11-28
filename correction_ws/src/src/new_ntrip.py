#!/usr/bin/env python3

import rospy
import socket
import base64
import datetime
import time
from std_msgs.msg import String
from sbg_driver.msg import SbgGpsPos

def load_credentials(file_path):
    credentials = {}
    with open(file_path, 'r') as file:
        for line in file:
            if '=' in line:
                key, value = line.strip().split('=', 1)
                credentials[key.strip()] = value.strip()
    return credentials

class NtripClient:
    def __init__(self, server, port, mountpoint, username, password):
        self.server = server
        self.port = port
        self.mountpoint = mountpoint
        self.username = username
        self.password = password
        self.rtcm_pub = rospy.Publisher('/ntrip/rtcm_data', String, queue_size=10)
        
        self.connect_to_ntrip()

    def connect_to_ntrip(self):
        credentials = f"{self.username}:{self.password}"
        encoded_credentials = base64.b64encode(credentials.encode('ascii')).decode('ascii')
        
        headers = (
            f"GET /{self.mountpoint} HTTP/1.1\r\n"
            f"Host: {self.server}\r\n"
            f"User-Agent: NTRIP JCMBsoftPythonClient/0.2\r\n"
            f"Authorization: Basic {encoded_credentials}\r\n"
            f"\r\n"
        )

        try:
            rospy.loginfo("Connecting to NTRIP server...")
            self.socket = socket.create_connection((self.server, self.port))
            self.socket.send(headers.encode('ascii'))

            response = self.socket.recv(4096).decode('utf-8')
            rospy.loginfo(f"Server response: {response}")

            if "ICY 200 OK" not in response and "HTTP/1.1 200 OK" not in response:
                rospy.logerr("NTRIP connection failed! Response: " + response)
                return

            rospy.loginfo("NTRIP connection established!")
            self.read_rtcm_data()

        except Exception as e:
            rospy.logerr(f"Failed to connect to NTRIP server: {e}")

    def read_rtcm_data(self):
        while not rospy.is_shutdown():
            try:
                data = self.socket.recv(4096)
                if data:
                    self.rtcm_pub.publish(data.hex())
                else:
                    rospy.logwarn("No data received, reconnecting...")
                    self.connect_to_ntrip()
                    break
            except socket.timeout:
                rospy.logwarn("Connection timed out. Reconnecting...")
                self.connect_to_ntrip()
                break
            except Exception as e:
                rospy.logerr(f"Error while receiving data: {e}")
                break

    def gps_pos_callback(self, msg):
        if hasattr(self, 'socket'):
            try:
                nmea_sentence = self.generate_gga_nmea(msg.latitude, msg.longitude, msg.altitude)
                if nmea_sentence:
                    self.socket.send((nmea_sentence + "\r\n").encode('utf-8'))
                    rospy.loginfo(f"NMEA Sentence Sent: {nmea_sentence}")
            except Exception as e:
                rospy.logerr(f"Error while sending NMEA data: {e}")

    def generate_gga_nmea(self, latitude, longitude, altitude):
        if latitude == 0.0 and longitude == 0.0:
            rospy.logwarn("Latitude and longitude are zero, skipping GGA generation.")
            return None

        flagN = "N" if latitude >= 0 else "S"
        flagE = "E" if longitude >= 0 else "W"

        latitude = abs(latitude)
        longitude = abs(longitude)

        lat_deg = int(latitude)
        lat_min = (latitude - lat_deg) * 60

        lon_deg = int(longitude)
        lon_min = (longitude - lon_deg) * 60

        now = datetime.datetime.utcnow()
        
        gga_string = (
            f"GPGGA,{now.hour:02d}{now.minute:02d}{now.second:04.2f},"
            f"{lat_deg:02d}{lat_min:07.4f},{flagN},"
            f"{lon_deg:03d}{lon_min:07.4f},{flagE},1,05,0.19,"
            f"{altitude:.1f},M,1.0,M,,"
        )

        checksum = self.calculate_checksum(gga_string)
        return f"${gga_string}*{checksum}"

    def calculate_checksum(self, string_to_check):
        xsum_calc = 0
        for char in string_to_check:
            xsum_calc ^= ord(char)
        return "%02X" % xsum_calc


if __name__ == '__main__':
    rospy.init_node('ntrip_client_node', anonymous=True)

    credentials = load_credentials('credentials.txt')

    ntrip_server = "78.24.131.136"
    ntrip_port = 2101
    mountpoint = "PRS30"
    username = credentials.get("username")
    password = credentials.get("password")

    client = NtripClient(ntrip_server, ntrip_port, mountpoint, username, password)

    rospy.Subscriber('/sbg/gps_pos', SbgGpsPos, client.gps_pos_callback)

    rospy.spin()
