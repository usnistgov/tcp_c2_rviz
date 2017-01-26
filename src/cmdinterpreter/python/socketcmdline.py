#!/usr/bin/env python
import socket   
import sys 
import math
import numpy as np
from numpy import matrix
import time
from xml.dom import minidom
import os.path
import tokenize
import numpy
import threading


# https://github.com/ros/geometry/blob/hydro-devel/tf/src/tf/transformations.py
import transformations

# ROS imports
import rospy
from sensor_msgs.msg import JointState

#holds the latest states obtained from joint_states messages
# http://wiki.ros.org/pr2_controllers/Tutorials/Getting%20the%20current%20joint%20angles
class LatestJointStates:

    def __init__(self):
        self.lock = threading.Lock()
        self.name = []
        self.position = []
        self.velocity = []
        self.effort = []
        self.thread = threading.Thread(target=self.joint_states_listener)
        self.thread.start()       

    #thread function: listen for joint_states messages
    def joint_states_listener(self):
        rospy.Subscriber('joint_states', JointState, self.joint_states_callback)
        rospy.spin()

    #callback function: when a joint_states message arrives, save the values
    def joint_states_callback(self, msg):
        self.lock.acquire()
        self.name = msg.name
        self.position = msg.position
        self.velocity = msg.velocity
        self.effort = msg.effort
        self.lock.release()
    def joint_states(self):
        position=[]
        for joint_name in self.name:
            index = self.name.index(joint_name)
            position.append( self.position[index])
        return position

# A..B assume p=[0..1]
def lerp(A,B,p):
    value = A + ((B-A) * p) 
    return value
    
def computeCoorindatedWaypoints(_curJts,  _goalJts,  dGap,  bAddStart):
    dMax = 0.0
   
    for  i in range (len(_curJts)):
        #print "Joints", _curJts[i],":" , _goalJts[i]
        dMax = max(dMax, abs(_curJts[i] - _goalJts[i]))

    nIncrement = int(dMax / dGap) # assume > 0 (+1?)
    joints=[]
    k = 1 if bAddStart else  0
    
    for i in range(k,nIncrement+1):
        jnts=[]
        for  j in range (len(_curJts)):
            jnts.append(lerp(_curJts[j], _goalJts[j], float( i) / float( nIncrement)))
        joints.append(jnts)
    return joints


class CrclClientSocket:
    def __init__(self, host, port):
        self.host=host
        self.port=port
        self.stopconnecting=False
        self.nextdata=''

    def connect(self):
        try:
            if(self.stopconnecting):
                return
            self.sock = socket.socket(
                    socket.AF_INET, socket.SOCK_STREAM)
            self.sock.connect((self.host, self.port))
        except socket.error, msg:
            print 'Failed to create socket. Error code: ' + str(msg[0]) + ' , Error message : ' + msg[1]
            time.sleep( 5 )
            self.connect()
   
    def disconnect(self):
        self.sock.close()
 
    def syncsend(self, msg):
        sent = self.sock.send(msg)
        if sent == 0:
            self.disconnect()
            self.connect()  
            # raise RuntimeError("socket connection broken")      
 
    # http://code.activestate.com/recipes/408859/

    def syncreceive(self, end):
        #total_data=[];
        self.End=end # '</CRCLStatus>'
        data=''
        alldata=self.nextdata
        while True:
                data=self.sock.recv(8192)
                if data == 0:
                    alldata='' # empty string
                    return 
                alldata=alldata+data
                if self.End in alldata:
                    alldata=alldata[:alldata.find(self.End)]
                    self.nextdata=data[data.find(self.End)+1:]
                    break
        return alldata  # ''.join(total_data)

def get_tokens(text):
    tokens = tokenize.tokenize(text)
    return tokens
 
def parse_token(tokens) :
    global mysocket
    try:
        if tokens[0] == "q":
            mysocket.syncsend("quit\n")
            return -1  # stops the loop
        elif tokens[0] == "quit":
           mysocket.syncsend("quit\n")
           return -1   # stops the loop
        elif tokens[0] == "sleep":
           time.sleep(float(tokens[1]))    
           
        else:
           tokens.append("\n")
           mysocket.syncsend(" ".join(tokens))
           
    except:
        pass    
    return 0
    
#time.sleep(10)
mysocket = CrclClientSocket("localhost", 31000)
print 'Socket Created'
mysocket.connect()
print 'Socket Connected'

rospy.init_node('rvizcli', anonymous=True)
rate = rospy.Rate(100) # 10hz
jts = LatestJointStates()
domark=True
# q  - quit &send quit
# quit  - quit send quit
while True:    # infinite loop
    try:
        msg = raw_input("> ")
        msgtokens=msg.split( )

        if msgtokens[0] == "file":
            f = open(os.getcwd()+'/'+msgtokens[1], 'r')
            #print 'file',os.getcwd()+'/'+msgtokens[1]
            line = f.readline()
            while line:
                    subtokens=line.split( )   
                    print 'subtokens',subtokens
                    parse_token(subtokens)
                    line = f.readline()
            f.close()
        elif msgtokens[0] == "where":
            print "Where=", jts.joint_states()
        elif msgtokens[0] == "domark":
            domark=False if domark else True
        elif msgtokens[0] == "goto":
            s=""
            subtokens=s.join(msgtokens[1:])  # in case there are spaces
            print "subtokens=", subtokens
            strgoaljts=subtokens.split(',')  
            goaljts= [float(i) for i in strgoaljts]
            print "goaljts=", goaljts
            # check current and goal joints same number?
            print "Where=", jts.joint_states()
            path=computeCoorindatedWaypoints(jts.joint_states(),  goaljts,  0.01,  1)
            for joints in path:
            	sjoints=[]
            	for e in joints:
            		sjoints.append("{:06.2f}".format(e) )
             	
           	print sjoints
                cmd =  'joints '+','.join(sjoints) + '\n'
                print "Cmd=",cmd
                mysocket.syncsend(cmd)
                if domark:
                   mysocket.syncsend("mark\n")
                time.sleep(float(0.1))
        else:
            b=parse_token(msgtokens)
            if(b<0):
                break
    except KeyboardInterrupt:
        break
    except rospy.ROSInterruptException:
        break
    except:
        break
       
mysocket.disconnect()        







