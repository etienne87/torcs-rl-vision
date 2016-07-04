import sys
import argparse
import socket
import os
import subprocess, signal
import time
import multiprocessing as mp

#here import your driver method
import msgParser
import carState
import carControl

#optional for image display
import cv2
import numpy as np

def recvall(sockt, count):
    buf = b''
    while count > 0:
        newbuf = sockt.recv(count)
        #if not newbuf: return None
        buf += newbuf
        count -= len(newbuf)
    return buf

"""
                    TorcsClient 

    - build scr_server simple client, receives image
    - takes a driver in argument

"""
class TorcsClient:
    def __init__(self, driver, host_ip='localhost', host_port=3001, id='SCR', verbose=False, max_steps=0, imgsize=(320,240)):
        self.id = id
        self.host_ip = host_ip
        self.host_port = host_port
        self.verbose = verbose
        self.max_steps = max_steps
        self.currentStep = 0
        self.imgsize = imgsize
        self.imsize = self.imgsize[0]*self.imgsize[1]*3
        self.sock = None
        self.driver = driver
        print('created bot @ : ',(self.host_ip,self.host_port))
        

    def connect_tcp(self):   
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        except socket.error, msg:
            print 'Could not make a socket.'
            time.sleep(1)
            self.connect_tcp()

        try:
             self.sock.connect((self.host_ip,self.host_port))
             self.sock.settimeout(1.0)
        except socket.error, msg:
            print('retry connect : ', msg)
            time.sleep(1)
            self.connect_tcp()

    def deconnect(self):
        if(self.sock != None):
            self.sock.close()
            self.sock = None

    def connect(self):
        self.connect_tcp()
        while True:
            print 'Sending id to server: ', self.id
            buf = self.id + self.driver.init()
            print 'Sending init string to server:', buf
            
            try:
                self.sock.sendto(buf, (self.host_ip, self.host_port) )
            except socket.error, msg:
                print "Failed to send data...Exiting..."
                time.sleep(1)
                self.connect() #retry?

            try:
                buf, addr = self.sock.recvfrom(1000)
            except socket.error, msg:
                print "didn't get response from server..."
        
            if buf.find('***identified***') >= 0:
                print 'Received: ', buf
                break

    def getState(self):
        shutdownClient = False
        # wait for an answer from server
        buf_in = None
        try:
            buf_in, addr = self.sock.recvfrom(1000)
        except socket.error, msg:
            print "didn't get response from server..."

        if self.verbose:
            print 'Received: ', buf_in
        
        if buf_in != None and buf_in.find('***shutdown***') >= 0:
            self.driver.onShutDown()
            shutdownClient = True
            print 'Client Shutdown'
        
        if buf_in != None and buf_in.find('***restart***') >= 0:
            self.driver.onRestart()
            print 'Client Restart'
        
        self.currentStep += 1
        if self.currentStep != self.max_steps:
            if buf_in != None:
                self.driver.state.setFromMsg(buf_in)
                imsize = self.driver.state.imsize
                msg = recvall(self.sock, imsize)
                if(len(msg) == imsize):
                    self.driver.state.setImage(msg)

    def sendAction(self, buf_out):
        if buf_out != None:
            try:
                self.sock.sendto(buf_out, (self.host_ip, self.host_port) )
            except socket.error, msg:
                self.connect_tcp()

    #split this in several stuff
    def control(self):
        shutdownClient = False
        # wait for an answer from server
        buf_in = None
        buf_out = None
        try:
            buf_in, addr = self.sock.recvfrom(1000)
        except socket.error, msg:
            print "didn't get response from server..."


        if self.verbose:
            print 'Received: ', buf_in
        
        if buf_in != None and buf_in.find('***shutdown***') >= 0:
            self.driver.onShutDown()
            shutdownClient = True
            print 'Client Shutdown'
        
        if buf_in != None and buf_in.find('***restart***') >= 0:
            self.driver.onRestart()
            print 'Client Restart'
        
        self.currentStep += 1
        if self.currentStep != self.max_steps:
            if buf_in != None:
                self.driver.state.setFromMsg(buf_in)
                imsize = self.driver.state.imsize
                msg = recvall(self.sock, imsize)
                if(len(msg) == imsize):
                    self.driver.state.setImage(msg)
                buf_out = self.driver.drive()
        else:
            buf_out = '(meta 1)'
        
        
        if self.verbose:
            print 'Sending: ', buf_out

        if buf_out != None:
            try:
                self.sock.sendto(buf_out, (self.host_ip, self.host_port) )
            except socket.error, msg:
                self.connect_tcp()
    
        return shutdownClient

class DriverActions:
    def __init__(self):
        self.parser = msgParser.MsgParser()
        self.state = carState.CarState()
        self.control = carControl.CarControl()
        self.steer_lock = 0.785398
        self.max_speed = 100
        self.prev_rpm = None
        self.steers = [-1.0, -0.8, -0.6, -0.5, -0.4, -0.3, -0.2, -0.15, -0.1, -0.05, 0.0, 0.05, 0.1, 0.15, 0.2, 0.3, 0.4, 0.5, 0.6, 0.8, 1.0]
        self.speeds = [-1.0, -0.5, 0.0, 0.5, 1.0] 
        self.num_steers = len(self.steers)  
        self.num_speeds = len(self.speeds)
        self.num_actions = self.num_steers = self.num_speeds
        
    def init(self):
        '''Return init string with rangefinder angles'''
        self.angles = [0 for x in range(19)]
        for i in range(5):
            self.angles[i] = -90 + i * 15
            self.angles[18 - i] = 90 - i * 15
        for i in range(5, 9):
            self.angles[i] = -20 + (i-5) * 5
            self.angles[18 - i] = 20 - (i-5) * 5
        return self.parser.stringify({'init': self.angles})

    def drive(self,action):
        #can only do one thing at a time (will adapt this later)
        if action > len(self.steers):
            steer = action - len(self.steers)
            self.setSteerAction(steer)
        else:
            speed = action
            self.setSpeedAction(speed)
        self.setGear()
        return self.control.toMsg()
            
    def setGear(self):
        rpm = self.state.getRpm()
        gear = self.state.getGear()
        if self.prev_rpm == None:
            up = True
        else:
            if (self.prev_rpm - rpm) < 0:
                up = True
            else:
                up = False
        if up and rpm > 7000:
            gear += 1
        if not up and rpm < 3000:
            gear -= 1
        self.control.setGear(gear)

    def setSteerAction(self, steer):
        assert 0 <= steer <= self.num_steers

    def setSpeedAction(self, speed):
        assert 0 <= speed <= self.num_speeds
        accel = self.speeds[speed]
        if accel >= 0:
            #print "accel", accel
            self.control.setAccel(accel)
            self.control.setBrake(0)
        else:
            #print "brake", -accel
            self.control.setAccel(0)
            self.control.setBrake(-accel)


import collections

# Need to adapt to general Environment Class
class TorcsEnv(object):
    def __init__(self, host_ip='localhost', host_port=3001, id='SCR', verbose=False, max_steps=0, imgsize=(320,240)):
        self.host_port = host_port
        self.driver = DriverActions()
        self.bot = TorcsClient(self.driver, host_ip,host_port,id,verbose,max_steps,imgsize)
        self.proc = None
        configname = "/home/valeodar/workspace/torcs/pyScrcClient/src/quickrace.xml"
        self.commandline = 'torcs ' + ' -p '+str(self.host_port) + '-nofuel -nodamage -nolaptime' + '&'
        self.reset()
        #first state
        self.bot.getState() #first prestate is not interesting

        self.reward = 0
        self.width = 80
        self.height = 60
        self.image = None

        self.n_last_screens = 4
        self.last_screens = collections.deque(
            [np.zeros((self.width, self.height), dtype=np.uint8)] * 4,
            maxlen=self.n_last_screens)

    def pause(self):
        subprocess.call("xte 'key p'", shell=True)

    def reset(self):
        self.bot.deconnect()
        if(self.proc is not None):
            sys.stdout.flush()
            os.killpg(os.getpgid(self.proc.pid), signal.SIGTERM)

        self.proc = subprocess.Popen(self.commandline, shell=True,stdin=subprocess.PIPE,\
                                                                  stdout=subprocess.PIPE,\
                                                                  preexec_fn=os.setsid) 
        
        for i in range(6):
            subprocess.call("xte 'key Return'", shell=True)
            subprocess.call("xte 'usleep 100000'", shell=True)

        print('START TORCS ENV')

        self.bot.connect()

    def __dell__(self):
        sys.stdout.flush()
        os.killpg(os.getpgid(self.proc.pid), signal.SIGTERM)

    def run(self):
        return self.bot.control()

    def step(self, action):
        
        msg = self.driver.drive(action)
        self.bot.sendAction(msg)
        self.bot.getState()

        terminal = self.driver.state.getTerminal()
        self.reward = self.driver.state.getReward(terminal)
        rgb_img = self.driver.state.getImage()
        gray = rgb_img[:, :, 0] * 0.2126 + rgb_img[:, :, 1] * 0.0722 + rgb_img[:, :, 2] * 0.7152

        gray = gray.astype(np.uint8)
        self.current_screen = cv2.resize(gray,(self.width,self.height),0,0,cv2.INTER_LINEAR)

        cv2.imshow('gray',self.current_screen)
        cv2.waitKey(1)

        if not terminal:
            self.last_screens.append(self.current_screen)

        return self.current_screen, self.reward, terminal, None

    @property
    def is_terminal(self):
        return self.driver.state.getTerminal()

    @property
    def state(self):
        assert len(self.last_screens) == 4
        return list(self.last_screens)

    def receive_action(self, action):
        rewards = []
        for i in range(4):
            im, r, terminal, _ = self.step(action)
            rewards.append(r)
            
            if self.is_terminal:
                break   
             
        self._reward = sum(rewards)
        
        return self._reward
