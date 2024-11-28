#!/usr/bin/env python3

import rospy
import socket
import base64
import datetime
import time
from std_msgs.msg import String
from sbg_driver.msg import SbgGpsRaw

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

    def nmea_callback(self, msg):
        if hasattr(self, 'socket'):
            try:
                nmea_sentence = self.format_nmea(msg.data)
                self.socket.send((nmea_sentence + "\r\n").encode('utf-8'))
                rospy.loginfo(f"NMEA Sentence Sent: {nmea_sentence}")
            except Exception as e:
                rospy.logerr(f"Error while sending NMEA data: {e}")

    def format_nmea(self, raw_data):
        try:
            nmea_sentence = ''.join([chr(byte) for byte in raw_data])
            return nmea_sentence
        except Exception as e:
            rospy.logerr(f"Error while formatting NMEA data: {e}")
            return ''

    def generate_dummy_nmea(self):
        latDeg, latMin, flagN, lonDeg, lonMin, flagE = self.set_position(50.09, 8.66)
        now = datetime.datetime.utcnow()
        gga_string = "GPGGA,%02d%02d%04.2f,%02d%07.4f,%s,%03d%07.4f,%s,1,05,0.19,+00400,M,1.000,M,," % (
            now.hour, now.minute, now.second, latDeg, latMin, flagN, lonDeg, lonMin, flagE)
        checksum = self.calculate_checksum(gga_string)
        return f"${gga_string}*{checksum}\r\n"

    def set_position(self, lat, lon):
        # Calculer les valeurs pour une trame NMEA
        flagN, flagE = "N", "E"
        if lat < 0:
            lat = -lat
            flagN = "S"
        if lon < 0:
            lon = -lon
            flagE = "W"

        latDeg = int(lat)
        latMin = (lat - latDeg) * 60
        lonDeg = int(lon)
        lonMin = (lon - lonDeg) * 60

        return latDeg, latMin, flagN, lonDeg, lonMin, flagE

    def calculate_checksum(self, string_to_check):
        xsum_calc = 0
        for char in string_to_check:
            xsum_calc ^= ord(char)
        return "%02X" % xsum_calc


if __name__ == '__main__':
    rospy.init_node('ntrip_client_node', anonymous=True)
    
    # Paramètres pour le serveur NTRIP
    ntrip_server = "78.24.131.136"
    ntrip_port = 2101
    mountpoint = "PRS30"
    username = "CESI_92" 
    password = "29052401"  

    client = NtripClient(ntrip_server, ntrip_port, mountpoint, username, password)

    rospy.Subscriber('/sbg/gps_raw', SbgGpsRaw, client.nmea_callback)

    rospy.spin()
