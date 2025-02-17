import serial 
from time import sleep
import datetime
import time
import os

# BAUD_RATE = 230400
BAUD_RATE = 921600
FP_COMPRESSION = (1000, 1000, 1000, 1000, 1000, -1000, -1000)
STATE = 0

import threading

# For broadcasting commands to the STM32
def broadcaster():
  state = 0
  while(True):
    match state:
      case 0:
        angles = (0, 0, 0, 0, 0, 0, 0)
        state = 1
      case 1:
        angles = (2, 2, 2, 2, 2, 2, 2)
        state = 0
      case _:
        state = 0

    # Get messages
    type = "POS"
    serial_send, serial_list = compressMsg(angles,type=type)

    # Send to microcontroller
    print(f"[SENT {type}]: {serial_list}")
    STM32_serial.write(serial_send)

    sleep(2)

def compressMsg(msg, type = "POS"):
  if type == "POS" :
    serial_list = ['p'.encode("utf-8")]
    serial_send = b'p'  
  elif type == "VEL":
    serial_list = ['v'.encode("utf-8")]
    serial_send = b'v'
  elif type == "STATE":
    serial_list = ['s'.encode("utf-8")]
    serial_send = b's'
  
  compressed_inputs = [int(msg[i] * FP_COMPRESSION[i]) for i in range(len(FP_COMPRESSION))]
  for i in range(0, len(compressed_inputs)):
    serial_list.append(compressed_inputs[i].to_bytes(2, 'big', signed=True))
    serial_send = serial_send + compressed_inputs[i].to_bytes(2, 'big', signed=True)

  return serial_send, serial_list
    
def convVal(value):
  try:
    return (float(value))/1000
  except ValueError:
    return 0



# Listender to print and log the STM32's outputs
def listener(STM32_serial, file_name):
  start_time = time.time()
  while True:
    if STM32_serial.in_waiting >= 40:
      read_val = STM32_serial.readline().decode('utf-8')
      if read_val == "":
        break
      # If it is sending encoder data
      if read_val[0] == 'e':
        data_str = read_val[1:-2].split(',')
        data_float = [convVal(i) for i in data_str]
        print(f"    [MSG] joint: {data_float[0:7]} | SEA: {data_float[7:]}")
        curr_time = time.time() - start_time

        f = open(file_name,'a')
        f.write(str(curr_time) + ", " + str(data_float)[1:-1] + "\n")
      else:
        print("    [Invalid]:", read_val, end="")
    

# Script to try and connect to STM32
known_hosts = ["D30JKY57", "DU0D4EM2"]
def getHost():
  print("Attempting to connect to STM32 Host")
  for host in known_hosts:
    try:
      print(f"  Attempting connect to host: /dev/tty.usbserial-{host}") 
      return serial.Serial(port=f'/dev/tty.usbserial-{host}', baudrate=BAUD_RATE, timeout=1)
    except:
      print(f"    [ERROR] Failed to connect to host") 
  return None



def getFileName():
  path = "scripts/data/output.csv"
  filename, extension = os.path.splitext(path)
  counter = 0

  # Look for an available file name
  path = filename + "(0)" + extension
  while os.path.exists(path):
    path = filename + "(" + str(counter) + ")" + extension
    counter += 1
  
  print( f"Output data to file: {path}")
  return path




if __name__ == '__main__':
    
  STM32_serial = getHost()
  if STM32_serial == None:
    raise Exception("Unable to detect STM32 controller")
  else:
    print("Connected to STM32 host successfully")

  file_name = getFileName()
  f = open(file_name,'w')
  f.write("")

  
  print( f"\n===================================\n\n")

  # Setup listener thread
  t1 = threading.Thread(target=listener, daemon=True, args=[STM32_serial, file_name])
  t1.start()
  
  # Run broadcaster
  broadcaster()


    