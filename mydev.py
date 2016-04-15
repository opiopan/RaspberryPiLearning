#!/usr/bin/python

import smbus
import time
import threading

# Define some device parameters

class I2CLCD:
  LCD_WIDTH = 16
  LCD_CHR = 1 # Mode - Sending data
  LCD_CMD = 0 # Mode - Sending command

  LCD_LINE_ADDR = [
    0x80, # LCD RAM address for the 1st line
    0xC0, # LCD RAM address for the 2nd line
    0x94, # LCD RAM address for the 3rd line
    0xD4 # LCD RAM address for the 4th line
    ]

  ENABLE = 0b00000100 # Enable bit

  E_PULSE = 0.0005
  E_DELAY = 0.0005

  def __init__(self, addr):
    self.I2C_ADDR  = addr # I2C device address
    self.LCD_BACKLIGHT  = 0x08  # On

    #Open I2C interface
    #bus = smbus.SMBus(0)  # Rev 1 Pi uses 0
    self.bus = smbus.SMBus(1) # Rev 2 Pi uses 1

    # Initialise display
    self.sendByte(0x33,I2CLCD.LCD_CMD) # 110011 Initialise
    self.sendByte(0x32,I2CLCD.LCD_CMD) # 110010 Initialise
    self.sendByte(0x06,I2CLCD.LCD_CMD) # 000110 Cursor move direction
    self.sendByte(0x0C,I2CLCD.LCD_CMD) # 001100 Disp On, Cursor Off, Blink Off 
    self.sendByte(0x28,I2CLCD.LCD_CMD) # 101000 Data length, lines, font sz
    self.sendByte(0x01,I2CLCD.LCD_CMD) # 000001 Clear display
    time.sleep(I2CLCD.E_DELAY)

  def clearDisplay(self):
    self.sendByte(0x01,I2CLCD.LCD_CMD) # 000001 Clear display
    time.sleep(I2CLCD.E_DELAY)

  def off(self):
    tmp = self.LCD_BACKLIGHT
    self.LCD_BACKLIGHT = 0
    self.clearDisplay()
    self.LCD_BACKLIGHT = tmp

  def sendByte(self, bits, mode):
    # Send byte to data pins
    # bits = the data
    # mode = 1 for data
    #        0 for command
    bits_high = mode | (bits & 0xF0) | self.LCD_BACKLIGHT
    bits_low = mode | ((bits<<4) & 0xF0) | self.LCD_BACKLIGHT

    # High bits
    self.bus.write_byte(self.I2C_ADDR, bits_high)
    self.toggleEnable(bits_high)

    # Low bits
    self.bus.write_byte(self.I2C_ADDR, bits_low)
    self.toggleEnable(bits_low)

  def toggleEnable(self, bits):
    # Toggle enable
    time.sleep(I2CLCD.E_DELAY)
    self.bus.write_byte(self.I2C_ADDR, (bits | I2CLCD.ENABLE))
    time.sleep(I2CLCD.E_PULSE)
    self.bus.write_byte(self.I2C_ADDR,(bits & ~I2CLCD.ENABLE))
    time.sleep(I2CLCD.E_DELAY)

  def sendString(self, line, message):
    # Send string to display

    message = message.ljust(I2CLCD.LCD_WIDTH," ")

    self.sendByte(I2CLCD.LCD_LINE_ADDR[line], I2CLCD.LCD_CMD)

    for i in range(I2CLCD.LCD_WIDTH):
      self.sendByte(ord(message[i]),I2CLCD.LCD_CHR)


class ThermoSensor(threading.Thread):
  THERMO_SENSOR_INTERVAL = 2.0

  def __init__(self, sensor_id):
    super(ThermoSensor, self).__init__()
    self.path = '/sys/bus/w1/devices/' + sensor_id + '/w1_slave'
    self.update()
    self.isUpdating = False
    self.stop = False
    self.start()

  def run(self):
    while not self.stop:
      time.sleep(ThermoSensor.THERMO_SENSOR_INTERVAL)
      self.isUpdating = True
      self.update()
      self.isUpdating = False
        
  def update(self):
    file = open(self.path, 'r')
    last = ''
    for line in file:
      last = line
    file.close()
    tstr = last.split('=')[-1]
    c = float(tstr) / 1000.0
    f = 9.0 / 5.0 * c + 32.0
    self.celcius = round(c, 1)
    self.fahrenheit = round(f, 1)

  def cleanup(self):
    self.stop = True
    self.join()
