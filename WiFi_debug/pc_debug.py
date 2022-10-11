import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
import collections
import socket

HOST = "192.168.4.1"  # The server's hostname or IP address
PORT = 25586  # The port used by the server

interval=80
sensor_timeout=0.02

print("connecting...")
s=socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.settimeout(3) # set timeout to 3 seconds
try:
    s.connect((HOST, PORT))
    s.sendall(b"test\n")
    data = s.recv(1024).decode()
    if(data=="OK"):
        print("connected!")
    else:
        print("invalid response to connect message")
        #exit()
except:
    print("failed to connect")
    #exit()

# Will only reach here if connected succesfully


def updateGraph(i):
    s.sendall(b"C\n")

    reading.popleft()
    reading.append(reading[-1])

    try:
        data = s.recv(20).decode()
    except:
        print("timeout")
        return
    if(data[-1]=='\n'):
        data=data.split("\n")[0]
        data=data.replace("\n", "")
        data=data.replace("\r", "")
    else:
        print("discarded data")
        return
    if(len(data)==0):
        print("empty message")
        return
    print(data)
    
    try:
        reading[-1]=float(data)
    except:
        print("data in invalid format")
    # clear axis
    ax.cla()
    ax.set_ylim([0, 1100])
    #plot
    ax.plot(reading)
    

reading= collections.deque(np.zeros(100))
fig = plt.figure(figsize=(12,6), facecolor='#DEDEDE')
ax = plt.subplot(121)
ax.set_facecolor('#DEDEDE')

s.sendall(b"C\n")
s.settimeout(sensor_timeout)
ani = FuncAnimation(fig, updateGraph, interval=interval)
plt.show()