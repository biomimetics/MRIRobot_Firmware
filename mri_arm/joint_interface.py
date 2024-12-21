import serial 
from time import sleep
import datetime
import time

# BAUD_RATE = 230400
BAUD_RATE = 921600

COUNTER_TRIGGER = 10000000
FP_COMPRESSION = (1000, 1000, 1000, 1000, 1000, 1000, 1000)
STATE = 0

FILE_NAME = "output.log"


# For out callback, we send a value between 0 and 10,000 
def callback(message):
  global STATE
  if STATE == 0:
    STATE = 1
    angles = (0, 0, 0, 0, 0, 0, 0)
  else:
    STATE = 0
    # angles = (0, -0.8, 0, 0.8, 0, 0, 0)
    angles = (2, 2, 2, 2, 2, 2, 2)

  # Send to microcontroller
  serial_list = ['p'.encode("utf-8")]
  
  compressed_inputs = [int(angles[i] * FP_COMPRESSION[i]) for i in range(len(FP_COMPRESSION))]
  
  serial_send = b''
  for i in range(0, len(compressed_inputs)):
    serial_list.append(compressed_inputs[i].to_bytes(2, 'big', signed=True))
    serial_send = serial_send + compressed_inputs[i].to_bytes(2, 'big', signed=True)
  
  print("send:", serial_list )
  STM32_serial.write('p'.encode("utf-8"))
  sleep(0.0205)
  
  STM32_serial.write(serial_send)
  
    
def conv_val(value):
  try:
    return (float(value))/1000
  except ValueError:
    return 0


# Define the method which contains the node's main functionality
def listener():
  start_time = time.time()
  counter = 0
  while True:
      if STM32_serial.in_waiting >= 40:
          read_val = STM32_serial.readline().decode('utf-8')
          if read_val == "":
            break
            
          # If it is sending encoder data
          if read_val[0] == 'e':
            data_str = read_val[1:-2].split(',')
            data_float = [conv_val(i) for i in data_str]
            print(data_float)

            curr_time = time.time() - start_time

            f = open(FILE_NAME,'a')
            f.write(str(curr_time) + ", " + str(data_float)[1:-1] + "\n")
          else:
            print("   [Invalid]:", read_val, end="")
      else:
        
        if counter >= COUNTER_TRIGGER:
          # callback(None)
          counter = 0
        else:
          counter += 1
    


if __name__ == '__main__':
    # Test different ports until one sticks. If not, rais an error
    try:
      STM32_serial = serial.Serial(port='/dev/tty.usbserial-D30JKY57', baudrate=BAUD_RATE, timeout=1)
      # STM32_serial = serial.Serial(port='/dev/tty.usbserial-DU0D4EM2', baudrate=BAUD_RATE, timeout=1)
    except:
       STM32_serial = None  
    
    if STM32_serial == None:
        raise Exception("Unable to detect STM32 controller")
    

    f = open(FILE_NAME,'w')
    f.write("")
    
    
    listener()
    