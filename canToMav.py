import can
import serial
from pymavlink import mavutil
import time
import os
import threading, queue, time
import copy
#Mavlink set up.
boot_time = time.time()
master = mavutil.mavlink_connection('tcp:127.0.0.1:5670') # For requesting MAV params.
#master = mavutil.mavlink_connection('tcp:0.0.0.0:5777')
mavOut = mavutil.mavlink_connection('udpout:192.168.2.2:14570', source_system=1, source_component=1) #For sending messages out to cockpit,.
mavOut.mav.ping_send(int((time.time() - boot_time) * 1e6), 0, 0, 0)
time.sleep(1)
mavOut.recv_match()


#Queues for message passsing.
rawCanQueue = queue.Queue(maxsize=512)
msgOutQueue = queue.Queue(maxsize=512)

#Canbus set up.
os.system("sudo killall slcand")
os.system("sudo slcand -o -c -s6 -S115200 /dev/ttyACM0 slcan0")
os.system("sudo ip link set slcan0 up")
hexIDs = {0x102,0x103,0x122,0x308,0x723} #Canbus msg IDs of interest.
# Configure the CAN bus interface
bytesToTemp = 60  #Canbus value for temperatures is 60 deg C lower than physical value.
bytesToRPM = 0.25 #Canbus 
CBDecoders = {#Functions to decode canbus messages based of sender ID.
    0x102: lambda d: {
        "CBThrot":    d[2],
        "CBEnCoolT":  d[3] - 60,
        "CBAirInT":   d[4] - 60,
    },

    0x103: lambda d: {
        "CBThrIN": d[0],
    },

    0x122: lambda d: {
        "CBRPM": (int.from_bytes(d[0:2],byteorder="big") * 0.25) - 1605.75 , #BE two byte value, scaled up by x4.
    },

    0x308: lambda d: {
        "CBHiTemp": 1 if d[0] >= 20 else 0, #Value 20 if high temp warning (>96 Deg C).
    },

    0x723: lambda d: {
        "CBFuel": round((860-int.from_bytes(d[5:7],byteorder="big"))/8,0)
    },
}

#Helper functions:
def getTime():
    return int((time.time() - boot_time) * 1e6)

#Function to flush canbus.
def flush_bus(bus): 
    # Drain all pending frames without blocking
    while True:
        msg = bus.recv(timeout=0)
        if msg is None:
            break  # buffer empty

def CBPuller():
    bus = can.interface.Bus(channel='slcan0', interface='socketcan', can_filters = [{"can_id": cid, "can_mask": 0x7FF} for cid in hexIDs])
    print(f"Connected to {bus.channel_info}")
    cycleTimeOut = 1 #How long the cycle will last before refreshing all the canids.
    while True:
        IDCopyList = set(hexIDs)
        loopStartTime = time.time() 
        flush_bus(bus)
        queryFuelLevel(bus)
        while time.time()-loopStartTime<cycleTimeOut:
            if not IDCopyList: #If nothing left in the list, wait until start of next cycle.
                remaining = cycleTimeOut - (time.time() - loopStartTime)
                if remaining > 0:
                    time.sleep(remaining)
                break
            else:
                queryFuelLevel(bus)
                msg = bus.recv(timeout=0.5)
                if msg is not None and msg.arbitration_id in IDCopyList:
                    #Remove the ID from the list so we don't get a double decode in a cycle.
                    IDCopyList.remove(msg.arbitration_id)
                    rawCanQueue.put(msg)
            time.sleep(0.05)
    
#Make an object key pairs in the decoded, pass it to the sending function which iterates through all params and sends.
def decodeCBMsg():
    while True:
        rawMsg = rawCanQueue.get()
        decoder = CBDecoders.get(rawMsg.arbitration_id) #Decoder will hold the correct decoding function based on the table.
        #if(rawMsg.arbitration_id==0x723):
            #print(rawMsg)
        if decoder is None:
            continue
        msgOutQueue.put(decoder(rawMsg.data)) #Runs the decoder on the data, returns the value.

def publishNamedFloat(): #A single CB message can contain multiple individual named float params, so iterate through its entirety.
    while True:
        msg = msgOutQueue.get()
        
        for msgName in msg:
            if msgName=="CBFuel":
                print(msg)
            mavOut.mav.named_value_float_send(getTime(), msgName.encode(), msg[msgName])


#Write funciton to send message to request fuel.
def queryFuelLevel(bus):
    #BUDS bombardier level query for polling fuel level from cluster.
    fuelQuery = can.Message(arbitration_id=0x713,
                            data=[0x04,0x02,0x21,0x02,0x55,0x55,0x55,0x55],
                            is_extended_id=False)
    bus.send(fuelQuery)#Single send implementation.

def safe(target): #Safe wrapper to complain if thread dies.
    def wrapper():
        try:
            target()
        except Exception as e:
            print(f"[ERROR] {target.__name__} crashed: {e}")
            raise
    return wrapper

master.mav.ping_send(int((time.time() - boot_time) * 1e6), 0, 0, 0)
#Pull message from can for decoding.
CBPullerThread = threading.Thread(target=safe(CBPuller), daemon=True)
#Thread for pulling in message from serial, then decoding and putting on queue.
decodeCBMsgThread = threading.Thread(target=safe(decodeCBMsg), daemon=True)
#Thread for mavlink sender.
publishNamedFloatThread = threading.Thread(target=safe(publishNamedFloat), daemon=True)

CBPullerThread.start()
decodeCBMsgThread.start()
publishNamedFloatThread.start()
print("working.")

try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    print("Shutting down…")