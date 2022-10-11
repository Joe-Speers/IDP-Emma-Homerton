# This code connects to the arduino to display debug data on a graph
# There are a number of modes that can be selected with keyboard keys, see below

# use pip to install these modules first:
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
import collections
import socket
import keyboard
from scipy import interpolate

HOST = "192.168.4.1"  # The server's hostname or IP address
PORT = 25586  # The port used by the server

interval=80 # interval between sending requests for data in ms. If graph is broken up then increase untill stable
mode_code="R" # code letter for what data to request (see below for possible values)

#connect to Arduino
print("connecting...")
s=socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.settimeout(3) # set timeout to 3 seconds
try:
    s.connect((HOST, PORT))
    s.sendall(b"test\n")# send a test message
    data = s.recv(1024).decode()
    if(data=="OK"):# check reply to test message
        print("connected!")
    else:
        print("invalid response to connect message")
        exit()
except:
    print("failed to connect")
    exit()

# Will only reach here if connected succesfully

#called every interval to update the graph
def updateGraph(i):
    global mode_code
    global reading

    #check for keybord press to change mode
    if keyboard.is_pressed('R'):
        print('Swithched to Raw Sensor Input')
        reading= collections.deque(np.zeros(100))
        mode_code="R"
    if keyboard.is_pressed('I'):
        print('Swithched to Integral')
        reading= collections.deque(np.zeros(100))
        mode_code="I"
    if keyboard.is_pressed('D'):
        print('Swithched to Derivitive')
        reading= collections.deque(np.zeros(100))
        mode_code="D"
    if keyboard.is_pressed('E'): 
        print('Swithched to Error')
        reading= collections.deque(np.zeros(100))
        mode_code="E"
    if keyboard.is_pressed('C'):
        print('Swithched to Correction')
        reading= collections.deque(np.zeros(100))
        mode_code="C"

    #send a request for the next reading
    s.sendall((mode_code+"\n").encode('utf-8'))

    #move all readings to the left and add a new reading (currently not set).
    #if reading is unsuccessful then this will remain as NaN and will not show on the graph
    reading.popleft()
    reading.append(np.nan)

    #try to read data from the arduino
    try:
        data = s.recv(20).decode()#read up to 20 characters
    except:
        print("timeout")
        return
    #check that all the data has been recieved and remove newline chars
    if(data[-1]=='\n'):
        data=data.split("\n")[0]
        data=data.replace("\n", "")
        data=data.replace("\r", "")
    else:
        print("discarded data")
        return
    #check message is not empty
    if(len(data)==0):
        print("empty message")
        return

    #print data to the console
    print(data)
    
    #try to read response
    try:
        reading[-1]=float(data)
    except:
        print("data in invalid format")

    # clear axis
    ax.cla()
    #set axis range and title depending on mode
    if(mode_code=="R"):
        ax.set_ylim([0, 1100])
        ax.set_title("Raw Sensor Input")
    if(mode_code=="I"):
        ax.set_ylim([-1, 1])
        ax.set_title("Integral")
    if(mode_code=="D"):
        ax.set_ylim([-30, 30])
        ax.set_title("Derivitive")
    if(mode_code=="E"):
        ax.set_ylim([-1, 1])
        ax.set_title("Error")
    if(mode_code=="C"):
        ax.set_ylim([-1, 1])
        ax.set_title("Correction")
    #plot graph
    ax.plot(reading)
    
#setup matplotlib graph
reading= collections.deque(np.zeros(100))
fig = plt.figure(figsize=(12,6), facecolor='#DEDEDE')
ax = plt.subplot(121)
ax.set_facecolor('#DEDEDE')

#send the initial request for data
s.sendall((mode_code+"\n").encode('utf-8'))
s.settimeout(0)#expect data instantly
#this sets up a background operation that continuosly calls updateGraph
ani = FuncAnimation(fig, updateGraph, interval=interval)
#start displaying the graph
plt.show()