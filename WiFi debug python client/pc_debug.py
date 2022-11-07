# This code connects to the arduino for WiFi debugging
# It has a console to display messages from the Arduino and a input field to send commands
# There is also a graph to display sensor readings. This has a number of modes that can be selected with keyboard keys

# ANY CHANGES TO THESE, please replicate in util.h
PURPOSE={
    0:"EXIT_START_BOX",
    1:"TRAVEL_TO_FAR_SIDE",
    2:"PICK_UP_BLOCK",
    3:"TRAVEL_TO_START_SIDE",
    4:"DROP_BLOCK",
    5:"RETURN_HOME"
}

LOCATION={
    0:"SQUARE",
    1:"DROPOFF_SIDE",
    2:"RAMP",
    3:"COLLECTION_SIDE",
    4:"CROSS",
    5:"BLOCK_COLLECTION_AREA",
    6:"TUNNEL",
    7:"RED_SQUARE",
    8:"GREEN_SQARE"
}

TASK={
    0:"STOPPED",
    1:"MOVE_FORWARD",
    2:"REVERSE",
    3:"TURN_LEFT",
    4:"TURN_RIGHT",
    5:"FOLLOW_LINE",
    6:"SLOW_SWEEP",
    7:"TURN_AROUND",
    8:"RECOVERY",
    9:"FINDING_BLOCK"
}

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
        #if not plotting, process the message
        if(not plotting):
            ProcessMessage(msg)

    root.after(1, update) # queue next update

def ToggleGraph():
    global plotting
    if(plotting): plotting=False
    else: plotting=True

def ReturnHome():
    ConsoleWrite("sending return home message")
    SendMessage("~H")

def SendMessage(command=""): # send a message to the arduino from the text input box (command_entry)
    if(command==""):#if message blank, get message from input field
        command=command_entry.get()
        if(command==""):
            return #ignore if message is blank
        command_entry.delete(0, tkinter.END) # clear entry box    
    s.sendall((command+"\n").encode('utf-8')) # encode and send data
    print("Sent: '"+command+"'")

def SendReset():#tell robot to reset to starting state and reset clock
    ConsoleWrite("Resetting robot")
    SendMessage("RESET")

def SendStop():#tell robot to stop (resets and sets s to -10000, effectivly halting the robot)
    ConsoleWrite("Stopping robot")
    SendMessage("STOP")

def ConsoleWrite(msg): # write a message to the console
    console_log.configure(state="normal")  # make GUI field editable
    console_log.insert("end", msg+"\n")  # write text to textbox
    console_log.see("end")  # scroll to end
    console_log.configure(state="disabled")  # make GUI field readonly

def ProcessMessage(msg):
    if(msg==""): return
    if(msg[0]=="!"):# indicates message is a state update
        if(msg[1]=="L"):
            if(int(msg[2:]) in LOCATION.keys()):
                location_status_txt.set("Location: "+LOCATION[int(msg[2:])])
            else:
                location_status_txt.set("Location: "+msg[2:])
            
        elif(msg[1]=="P"):
            if(int(msg[2:]) in PURPOSE.keys()):
                purpose_status_txt.set("Purpose: "+PURPOSE[int(msg[2:])])
            else:
                purpose_status_txt.set("Purpose: "+msg[2:])
        elif(msg[1]=="T"):
            if(int(msg[2:]) in TASK.keys()):
                task_status_txt.set("Task: "+TASK[int(msg[2:])])
            else:
                task_status_txt.set("Task: "+msg[2:])
        elif(msg[1]=="R"):
            isLost_status_txt.set("Is Lost: "+msg[2:])
        elif(msg[1]=="C"):
            task_countdown_status_txt.set("Task countdown timer: "+msg[2:])
        elif(msg[1]=="S"):
            task_stopwatch_status_txt.set("Task stopwatch: "+msg[2:])
        elif(msg[1]=="J"):
            junction_counter_status_txt.set("Junction counter: "+msg[2:])
    else: #if message is not a state update, then print to console
        ConsoleWrite(msg)

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
            ProcessMessage(msg)# message is not a number so is probably a message instead
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
main_font=("consolas", "14", "normal")
ToggleBut = tkinter.Button(root, text ="Toggle graph", command = ToggleGraph,font=main_font)
ResetBut=tkinter.Button(root, text='RESET robot', command=SendReset,font=main_font)
StopBut=tkinter.Button(root, text='STOP robot', command=SendStop,font=main_font)
ReturnHome=tkinter.Button(root, text='RETURN HOME', command=ReturnHome,font=main_font)
command_entry = tkinter.Entry(root,font=main_font)
SubmitBut=tkinter.Button(root, text='Send Command', command=SendMessage,font=main_font)
FPS_text = tkinter.StringVar()
FPS_label = tkinter.Label( root, textvariable=FPS_text,font=main_font)
main_label = tkinter.Label( root, text="Wifi Debug",font=("consolas", "20", "normal"))
console_log = ScrolledText(root, height=30,font=main_font)
#status labels
location_status_txt = tkinter.StringVar()
purpose_status_txt = tkinter.StringVar()
task_status_txt = tkinter.StringVar()
isLost_status_txt = tkinter.StringVar()
task_countdown_status_txt = tkinter.StringVar()
task_stopwatch_status_txt = tkinter.StringVar()
junction_counter_status_txt = tkinter.StringVar()
location_status = tkinter.Label( root, textvariable=location_status_txt,font=main_font)
purpose_status = tkinter.Label( root, textvariable=purpose_status_txt,font=main_font)
task_status = tkinter.Label( root, textvariable=task_status_txt,font=main_font)
isLost_status = tkinter.Label( root, textvariable=isLost_status_txt,font=main_font)
task_countdown_status = tkinter.Label( root, textvariable=task_countdown_status_txt,font=main_font)
task_stopwatch_status = tkinter.Label( root, textvariable=task_stopwatch_status_txt,font=main_font)
junction_counter_status = tkinter.Label( root, textvariable=junction_counter_status_txt,font=main_font)
#the order of these commands determines position of GUI elements
main_label.pack(side=tkinter.TOP)
console_log.pack(side=tkinter.RIGHT)
FPS_label.pack()
ToggleBut.pack()
ResetBut.pack()
StopBut.pack()
ReturnHome.pack()
command_entry.pack()
SubmitBut.pack()
location_status.pack()
purpose_status.pack()
task_status.pack()
isLost_status.pack()
task_countdown_status.pack()
task_stopwatch_status.pack()
junction_counter_status.pack()
canvas.get_tk_widget().pack(expand=1)

#this sets up a background operation that continuosly calls updateGraph()
ani = FuncAnimation(fig, updateGraph, interval=interval)

root.after(1, update) #start update loop. This is called by Tkinter as often as possible
#start displaying the GUI window
root.mainloop()
