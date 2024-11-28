import base64
import socket
import datetime
import time

#dummyNMEA = "$GPGGA,143741.356,7839.493,S,07627.626,W,0,00,,,M,,M,,*45"
dummyNMEA = "$GPGGA,143741.356,7839.493,S,07627.626,W,0,00,,,M,,M,,*45\r\n"
my_host="78.24.131.136"
username = "CESI92" #"ESIGELEC"    #username for RTCM correction service
password = "29052401" #"18021905"    #password for RTCM correction service
port = 2101    #port for the service
#mountPoint="FKP30"
#mountPoint="MAC30"
#mountPoint="VRS30"
mountPoint="PRS30"#a du mal

'''Generate an encoding of the username:password for the service.
The string must be first encoded in ascii to be correctly parsed by the    
base64.b64encode function.'''
pwd = base64.b64encode("{}:{}".format(username, password).encode('ascii'))

#The following decoding is necessary in order to remove the b' character that
#the ascii encoding add. Othrewise said character will be sent to the net and misinterpreted.
pwd = pwd.decode('ascii')

print("Header sending... \n")

header =\
"GET /"+str(mountPoint)+" HTTP/1.1\r\n" +\
"Host my_host\r\n" +\
"Ntrip-Version: Ntrip/1.0\r\n" +\
"User-Agent: ntrip.py/0.1\r\n" +\
"Accept: */*\r\n" +\
"Connection: close\r\n" +\
"Authorization: Basic {}\r\n\r\n".format(pwd)#RVNJR0VMRUM6SUJT #RVNJR0VMRUM6MTgwMjE5MDU=

header =\
"GET /"+str(mountPoint)+" HTTP/1.1\n" +\
"User-Agent: NTRIP JCMBsoftPythonClient/0.2\n" +\
"Authorization: Basic {}\n\n".format(pwd)#RVNJR0VMRUM6SUJT #RVNJR0VMRUM6MTgwMjE5MDU=

"""
GET /FKP30 HTTP/1.1
User-Agent: NTRIP JCMBsoftPythonClient/0.2
Authorization: Basic RVNJR0VMRUM6MTgwMjE5MDU=

"""

def setPosition( lat, lon):
    flagN="N"
    flagE="E"
    if lon>180:
        lon=(lon-360)*-1
        flagE="W"
    elif (lon<0 and lon>= -180):
        lon=lon*-1
        flagE="W"
    elif lon<-180:
        lon=lon+360
        flagE="E"
    else:
        lon=lon
    if lat<0:
        lat=lat*-1
        flagN="S"
    lonDeg=int(lon)
    latDeg=int(lat)
    lonMin=(lon-lonDeg)*60
    latMin=(lat-latDeg)*60
    return latDeg,latMin,flagN,lonDeg,lonMin,flagE

def calcultateCheckSum( stringToCheck):
    xsum_calc = 0
    for char in stringToCheck:
        xsum_calc = xsum_calc ^ ord(char)
    return "%02X" % xsum_calc

def getGGAString():
    height=1212
    latDeg,latMin,flagN,lonDeg,lonMin,flagE = setPosition( 50.09, 8.66)
    now = datetime.datetime.utcnow()
    ggaString= "GPGGA,%02d%02d%04.2f,%02d%011.8f,%1s,%03d%011.8f,%1s,1,05,0.19,+00400,M,%5.3f,M,," % \
            (now.hour,now.minute,now.second,latDeg,latMin,flagN,lonDeg,lonMin,flagE,height)
    checksum = calcultateCheckSum(ggaString)
   
    #print  "$%s*%s\r\n" % (ggaString, checksum)
    return ("$%s*%s\r\n" % (ggaString, checksum))

while True:

    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    s.connect((my_host,int(port)))
    s.settimeout(10)

    print ("connect done")
    s.send(header.encode('ascii'))
    print ("send done\n"+str(header.encode('ascii')))

    print("Waiting answer...\n")
    #data = s.recv(2048).decode('ascii')
    casterResponse=s.recv(4096).decode('utf-8') #check utf-8 All the data
    header_lines = casterResponse.split("\r\n")

    print("casterResponse="+str(casterResponse))

    isConnected=False
    if casterResponse.find("SOURCETABLE")>=0:
       print("Mount point does not exist")
    elif casterResponse.find("401 Unauthorized")>=0:
       print("Unauthorized request\n")
    elif casterResponse.find("404 Not Found")>=0:
       print("Mount Point does not exist\n")
    elif casterResponse.find("ICY 200 OK")>=0:
       #Request was valid
       isConnected=True
    elif casterResponse.find("HTTP/1.0 200 OK")>=0:
       isConnected=True
    elif casterResponse.find("HTTP/1.1 200 OK")>=0:
       isConnected=True

    if isConnected:
        data = "Initial data"
        while data:
            try:
                data=s.recv(4096)
                print(data)
            except socket.timeout:
                print('Connection TimedOut\n')
                data=False
            except socket.error:
                print('Connection Error\n')
                data=False

            print ("send nmea"+str(getGGAString()))
            s.send(getGGAString().encode())

    #data = s.recv(4096)#.decode('ascii')
    #print(data)

    s.close()
    time.sleep(0.5)
