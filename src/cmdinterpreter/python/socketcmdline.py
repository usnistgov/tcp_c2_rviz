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
        else:
            b=parse_token(msgtokens)
            if(b<0):
                break
    except KeyboardInterrupt:
        break
    except:
        break
       
mysocket.disconnect()        







