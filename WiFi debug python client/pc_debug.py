# This code connects to the arduino for WiFi debugging
# It has a console to display messages from the Arduino and a input field to send commands
# There is also a graph to display sensor readings. This has a number of modes that can be selected with keyboard keys

# use pip to install these modules first:
import matplotlib.pyplot as plt
import datetime
import tkinter
from tkinter.scrolledtext import ScrolledText
from matplotlib.backends.backend_tkagg import (
    FigureCanvasTkAgg, NavigationToolbar2Tk)
from matplotlib.figure import Figure
import numpy as np
from matplotlib.animation import FuncAnimation
import collections
import socket
import keyboard

HOST = "192.168.4.1"  # The Arduino's IP address
PORT = 25586  # The port used by the server

interval=1#delay between sending graph data requests in ms
mode_code="R" # code letter for what sensor reading to request (see below for possible values)

plotting=False #true if currently plotting a graph

data_in="" #stores a fragment of a message as it is recieved
msg="" #stores the last received complete message

frames=0 #counter for the number of graph data points recieved each second, used to calculate fps
last_time=datetime.datetime.now() #stores the current time, used for timing in update()

### Functions ###

def _quit(): #called when GUI window closed
    print("quitting")
    s.close() # close connection to Arduino
    root.quit()     # stops GUI mainloop
    root.destroy()

def update(): #called as often as possible, every ~10ms. Handles all update events and communication with arduino.
    global frames
    global last_time
    global mode_code
    global reading
    global data_in
    global msg

    ### TIMER code

    elapsed = datetime.datetime.now()-last_time #calculate time elapsed since last update()
    if(elapsed.seconds>0):
        last_time=datetime.datetime.now()
        FPS_text.set("FPS: "+str(frames)) #calculate FPS if currently plotting a graph
        frames=0

    ### Keyboard press events

    #check for keybord press to change graph plotting mode
    if keyboard.is_pressed('R'):
        print('Swithched to Raw Sensor Input')
        reading= collections.deque(np.zeros(100))
        mode_code="R"
    if keyboard.is_pressed('I'):
        print('Swithched to Integral')
        reading= collections.deque(np.zeros(100))
        mode_code="I"
    if keyboard.is_pressed('D'):
        print('Swithched to derivative')
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

    ### Read data from Ardiono

    try:
        raw_data=s.recv(10)# try to recieve up to 10 characters
        if(len(raw_data)>0):
            data_in += raw_data.decode() #decode into string and add message fragment to data_in
    except:
        pass # ignore errors
    #check if a complete message has been recieved (indicated by a newline character)
    if('\n' in data_in):
        msg=data_in.split("\n")[0] #get the message portion
        data_in=data_in.split("\n",1)[1] #place the fragement of the next message back into data_in
        #remove newline characters
        msg=msg.replace("\n", "")
        msg=msg.replace("\r", "")
        #if not plotting, display the message in the console
        if(not plotting):
            ConsoleWrite(msg)

    root.after(1, update) # queue next update

def ToggleGraph():
    global plotting
    if(plotting): plotting=False
    else: plotting=True

def SendMessage(): # send a message to the arduino from the text input box (command_entry)
    command=command_entry.get()
    if(command!=""):
        s.sendall((command+"\n").encode('utf-8')) # encode and send data
        print("Sent: '"+command+"'")
        command_entry.delete(0, tkinter.END) # clear entry box

def ConsoleWrite(msg): # write a message to the console
    console_log.configure(state="normal")  # make GUI field editable
    console_log.insert("end", msg+"\n")  # write text to textbox
    console_log.see("end")  # scroll to end
    console_log.configure(state="disabled")  # make GUI field readonly

#called every interval to update the graph
def updateGraph(i):
    if(not plotting): return
    global frames
    global msg
    #send a request for the next reading
    s.sendall((mode_code+"\n").encode('utf-8'))

    #move all readings to the left and add a new reading (currently not set).
    #if reading is unsuccessful then this will remain as NaN and will not show on the graph
    reading.popleft()
    reading.append(np.nan)
    
    #try to read response
    if(msg!=""):
        try:
            reading[-1]=float(msg) # try to plot the reading on the graph
            frames+=1 # counter to calculate fps
            msg=""
        except:
            ConsoleWrite(msg)# message is not a number so is probably a message instead, so print to console
    else:
        print("TIMEOUT, Arduino may have lost connection")

    # clear axis
    ax.cla()
    #set axis range and title depending on mode
    if(mode_code=="R"):
        ax.set_ylim([0, 1100])
        ax.set_title("Raw Sensor Input")
    if(mode_code=="I"):
        ax.set_ylim([-0.3, 0.3])
        ax.set_title("Integral")
    if(mode_code=="D"):
        ax.set_ylim([-5, 5])
        ax.set_title("derivative")
    if(mode_code=="E"):
        ax.set_ylim([-1, 1])
        ax.set_title("Error")
    if(mode_code=="C"):
        ax.set_ylim([-1, 1])
        ax.set_title("Correction")
    #re-plot graph
    ax.plot(reading)

### Startup Code ###

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

s.settimeout(0)#expect data instantly from now on

## Will only reach here if connected succesfully

#setup GUI Window
root = tkinter.Tk()
root.wm_title("Wifi Debug")
root.protocol("WM_DELETE_WINDOW", _quit) # setup handle in case the window is closed

#setup matplotlib graph
reading= collections.deque(np.zeros(100))
fig = plt.figure(figsize=(12,6), facecolor='#DEDEDE')
ax = plt.subplot(121)
ax.set_facecolor('#DEDEDE')
#create canvas for graph
canvas = FigureCanvasTkAgg(fig, master=root)
canvas.draw()

#setup GUI buttons and text fields
ToggleBut = tkinter.Button(root, text ="Toggle graph", command = ToggleGraph)
command_entry = tkinter.Entry(root)
SubmitBut=tkinter.Button(root, text='Send Command', command=SendMessage)
FPS_text = tkinter.StringVar()
FPS_label = tkinter.Label( root, textvariable=FPS_text)
main_label = tkinter.Label( root, text="Wifi Debug",font=("consolas", "20", "normal"))
console_log = ScrolledText(root, height=30, font=("consolas", "12", "normal"))
#the order of these commands determines position of GUI elements
main_label.pack(side=tkinter.TOP)
console_log.pack(side=tkinter.RIGHT)
canvas.get_tk_widget().pack(side=tkinter.TOP,expand=1)
FPS_label.pack()
ToggleBut.pack()
command_entry.pack()
SubmitBut.pack()

#this sets up a background operation that continuosly calls updateGraph()
ani = FuncAnimation(fig, updateGraph, interval=interval)

root.after(1, update) #start update loop. This is called by Tkinter as often as possible
#start displaying the GUI window
root.mainloop()
