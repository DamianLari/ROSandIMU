#!/usr/bin/env python3

import rospy
import socket
import base64
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
        encoded_credentials = base64.b64encode(credentials.encode('utf-8')).decode('utf-8')
        
        headers = (
            f"GET /{self.mountpoint} HTTP/1.0\r\n"
            f"Host: {self.server}\r\n"
            f"Ntrip-Version: Ntrip/2.0\r\n"
            f"User-Agent: NTRIP Client/1.0\r\n"
            f"Authorization: Basic {encoded_credentials}\r\n"
            f"\r\n"
        )

        try:
            rospy.loginfo("Connecting to NTRIP server...")
            sock = socket.create_connection((self.server, self.port))
            sock.send(headers.encode('utf-8'))
            response = sock.recv(1024)
            
            if b"ICY 200 OK" not in response:
                rospy.logerr("NTRIP connection failed!")
                return

            rospy.loginfo("NTRIP connection established!")
            self.socket = sock
            
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
            except Exception as e:
                rospy.logerr(f"Error while receiving data: {e}")
                break

    def nmea_callback(self, msg):
        if hasattr(self, 'socket'):
            try:
                nmea_sentence = self.format_nmea(msg)
                self.socket.send((nmea_sentence + "\r\n").encode('utf-8'))
            except Exception as e:
                rospy.logerr(f"Error while sending NMEA data: {e}")

    def format_nmea(self, msg):
        try:
            nmea_sentence = ''.join([chr(byte) for byte in raw_data])
            return nmea_sentence
        except Exception as e:
            rospy.logerr(f"Error while formatting NMEA data: {e}")
            return ''

if __name__ == '__main__':
    rospy.init_node('ntrip_client_node', anonymous=True)
    
    ntrip_server = "78.24.131.136"
    ntrip_port = 2101
    mountpoint = "PRS30"
    username = "CESI_92" 
    password = "29052401"  


    client = NtripClient(ntrip_server, ntrip_port, mountpoint, username, password)
    
    rospy.Subscriber('/sbg/gps_raw', SbgGpsRaw, client.nmea_callback)

    rospy.spin()
