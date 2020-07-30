#!/usr/bin/python3
# @ made by d-rez / dark_skeleton
# Requires:
# - ADS1015 with Vbat on A0
# - pngview
# - a symbolic link to ic_battery_alert_red_white_36dp.png under
#   material_design_icons_master/device/drawable-mdpi/
# - an entry in crontab
# - material_design_icons_master github clone
# - some calibration, there's a lot of jitter
# - code comments. someday...

import time
# import Adafruit_ADS1x15
import subprocess
import os
import re
import logging
import logging.handlers
from datetime import datetime
from statistics import median
from collections import deque
from enum import Enum
import smbus
import time
import os

pngview_path="/usr/local/bin/pngview"
pngview_call=[pngview_path, "-d", "0", "-b", "0x0000", "-n", "-l", "15000", "-y", "0", "-x"]

iconpath="/home/pi/src/material-design-icons-master/device/drawable-mdpi/"
iconpath2 = os.path.dirname(os.path.realpath(__file__)) + "/overlay_icons/"
logfile = os.path.dirname(os.path.realpath(__file__)) + "/overlay.log"
dpi=36

env_icons = {
  "under-voltage": iconpath2+"flash.png",
  "freq-capped":   iconpath2+"thermometer.png",
  "throttled":     iconpath2+"thermometer-lines.png"
}
wifi_icons = {
  "connected": iconpath + "ic_network_wifi_white_"      + str(dpi) + "dp.png",
  "disabled":  iconpath + "ic_signal_wifi_off_white_"   + str(dpi) + "dp.png",
  "enabled":   iconpath + "ic_signal_wifi_0_bar_white_" + str(dpi) + "dp.png"
}
bt_icons = {
  "enabled":   iconpath + "ic_bluetooth_white_"           + str(dpi) + "dp.png",
  "connected": iconpath + "ic_bluetooth_connected_white_" + str(dpi) + "dp.png",
  "disabled":  iconpath + "ic_bluetooth_disabled_white_"  + str(dpi) + "dp.png"
}
icon_battery_critical_shutdown = iconpath2 + "alert-outline-red.png"

wifi_carrier = "/sys/class/net/wlan0/carrier" # 1 when wifi connected, 0 when disconnected and/or ifdown
wifi_linkmode = "/sys/class/net/wlan0/link_mode" # 1 when ifup, 0 when ifdown
bt_devices_dir="/sys/class/bluetooth"
env_cmd="vcgencmd get_throttled"

fbfile="tvservice -s"

#charging no load: 4.85V max (full bat)
#charging es load: 4.5V max

vmax = {"discharging": 8.136,
        "charging"   : 8.136 }
vmin = {"discharging": 6.112,
        "charging"   : 6.336 }
icons = { "discharging": [ "alert_red", "alert", "20", "30", "30", "50", "60",
                           "60", "80", "90", "full", "full" ],
          "charging"   : [ "charging_20", "charging_20", "charging_20",
                           "charging_30", "charging_30", "charging_50",
                           "charging_60", "charging_60", "charging_80",
                           "charging_90", "charging_full", "charging_full" ]}

# Config Register (R/W)
_REG_CONFIG                 = 0x00
# SHUNT VOLTAGE REGISTER (R)
_REG_SHUNTVOLTAGE           = 0x01

# BUS VOLTAGE REGISTER (R)
_REG_BUSVOLTAGE             = 0x02

# POWER REGISTER (R)
_REG_POWER                  = 0x03

# CURRENT REGISTER (R)
_REG_CURRENT                = 0x04

# CALIBRATION REGISTER (R/W)
_REG_CALIBRATION            = 0x05

class BusVoltageRange:
    """Constants for ``bus_voltage_range``"""
    RANGE_16V               = 0x00      # set bus voltage range to 16V
    RANGE_32V               = 0x01      # set bus voltage range to 32V (default)

class Gain:
    """Constants for ``gain``"""
    DIV_1_40MV              = 0x00      # shunt prog. gain set to  1, 40 mV range
    DIV_2_80MV              = 0x01      # shunt prog. gain set to /2, 80 mV range
    DIV_4_160MV             = 0x02      # shunt prog. gain set to /4, 160 mV range
    DIV_8_320MV             = 0x03      # shunt prog. gain set to /8, 320 mV range

class ADCResolution:
    """Constants for ``bus_adc_resolution`` or ``shunt_adc_resolution``"""
    ADCRES_9BIT_1S          = 0x00      #  9bit,   1 sample,     84us
    ADCRES_10BIT_1S         = 0x01      # 10bit,   1 sample,    148us
    ADCRES_11BIT_1S         = 0x02      # 11 bit,  1 sample,    276us
    ADCRES_12BIT_1S         = 0x03      # 12 bit,  1 sample,    532us
    ADCRES_12BIT_2S         = 0x09      # 12 bit,  2 samples,  1.06ms
    ADCRES_12BIT_4S         = 0x0A      # 12 bit,  4 samples,  2.13ms
    ADCRES_12BIT_8S         = 0x0B      # 12bit,   8 samples,  4.26ms
    ADCRES_12BIT_16S        = 0x0C      # 12bit,  16 samples,  8.51ms
    ADCRES_12BIT_32S        = 0x0D      # 12bit,  32 samples, 17.02ms
    ADCRES_12BIT_64S        = 0x0E      # 12bit,  64 samples, 34.05ms
    ADCRES_12BIT_128S       = 0x0F      # 12bit, 128 samples, 68.10ms

class Mode:
    """Constants for ``mode``"""
    POWERDOW                = 0x00      # power down
    SVOLT_TRIGGERED         = 0x01      # shunt voltage triggered
    BVOLT_TRIGGERED         = 0x02      # bus voltage triggered
    SANDBVOLT_TRIGGERED     = 0x03      # shunt and bus voltage triggered
    ADCOFF                  = 0x04      # ADC off
    SVOLT_CONTINUOUS        = 0x05      # shunt voltage continuous
    BVOLT_CONTINUOUS        = 0x06      # bus voltage continuous
    SANDBVOLT_CONTINUOUS    = 0x07      # shunt and bus voltage continuous


class INA219:
    def __init__(self, i2c_bus=1, addr=0x40):
        self.bus = smbus.SMBus(i2c_bus);
        self.addr = addr

        # Set chip to known config values to start
        self._cal_value = 0
        self._current_lsb = 0
        self._power_lsb = 0
        self.set_calibration_32V_2A()

    def read(self,address):
        data = self.bus.read_i2c_block_data(self.addr, address, 2)
        return ((data[0] * 256 ) + data[1])

    def write(self,address,data):
        temp = [0,0]
        temp[1] = data & 0xFF
        temp[0] =(data & 0xFF00) >> 8
        self.bus.write_i2c_block_data(self.addr,address,temp)

    def set_calibration_32V_2A(self):
        """Configures to INA219 to be able to measure up to 32V and 2A of current. Counter
           overflow occurs at 3.2A.
           ..note :: These calculations assume a 0.1 shunt ohm resistor is present
        """
        # By default we use a pretty huge range for the input voltage,
        # which probably isn't the most appropriate choice for system
        # that don't use a lot of power.  But all of the calculations
        # are shown below if you want to change the settings.  You will
        # also need to change any relevant register settings, such as
        # setting the VBUS_MAX to 16V instead of 32V, etc.

        # VBUS_MAX = 32V             (Assumes 32V, can also be set to 16V)
        # VSHUNT_MAX = 0.32          (Assumes Gain 8, 320mV, can also be 0.16, 0.08, 0.04)
        # RSHUNT = 0.1               (Resistor value in ohms)

        # 1. Determine max possible current
        # MaxPossible_I = VSHUNT_MAX / RSHUNT
        # MaxPossible_I = 3.2A

        # 2. Determine max expected current
        # MaxExpected_I = 2.0A

        # 3. Calculate possible range of LSBs (Min = 15-bit, Max = 12-bit)
        # MinimumLSB = MaxExpected_I/32767
        # MinimumLSB = 0.000061              (61uA per bit)
        # MaximumLSB = MaxExpected_I/4096
        # MaximumLSB = 0,000488              (488uA per bit)

        # 4. Choose an LSB between the min and max values
        #    (Preferrably a roundish number close to MinLSB)
        # CurrentLSB = 0.0001 (100uA per bit)
        self._current_lsb = .1  # Current LSB = 100uA per bit

        # 5. Compute the calibration register
        # Cal = trunc (0.04096 / (Current_LSB * RSHUNT))
        # Cal = 4096 (0x1000)

        self._cal_value = 4096

        # 6. Calculate the power LSB
        # PowerLSB = 20 * CurrentLSB
        # PowerLSB = 0.002 (2mW per bit)
        self._power_lsb = .002  # Power LSB = 2mW per bit

        # 7. Compute the maximum current and shunt voltage values before overflow
        #
        # Max_Current = Current_LSB * 32767
        # Max_Current = 3.2767A before overflow
        #
        # If Max_Current > Max_Possible_I then
        #    Max_Current_Before_Overflow = MaxPossible_I
        # Else
        #    Max_Current_Before_Overflow = Max_Current
        # End If
        #
        # Max_ShuntVoltage = Max_Current_Before_Overflow * RSHUNT
        # Max_ShuntVoltage = 0.32V
        #
        # If Max_ShuntVoltage >= VSHUNT_MAX
        #    Max_ShuntVoltage_Before_Overflow = VSHUNT_MAX
        # Else
        #    Max_ShuntVoltage_Before_Overflow = Max_ShuntVoltage
        # End If

        # 8. Compute the Maximum Power
        # MaximumPower = Max_Current_Before_Overflow * VBUS_MAX
        # MaximumPower = 3.2 * 32V
        # MaximumPower = 102.4W

        # Set Calibration register to 'Cal' calculated above
        self.write(_REG_CALIBRATION,self._cal_value)

        # Set Config register to take into account the settings above
        self.bus_voltage_range = BusVoltageRange.RANGE_32V
        self.gain = Gain.DIV_8_320MV
        self.bus_adc_resolution = ADCResolution.ADCRES_12BIT_32S
        self.shunt_adc_resolution = ADCResolution.ADCRES_12BIT_32S
        self.mode = Mode.SANDBVOLT_CONTINUOUS
        self.config = self.bus_voltage_range << 13 | \
                      self.gain << 11 | \
                      self.bus_adc_resolution << 7 | \
                      self.shunt_adc_resolution << 3 | \
                      self.mode
        self.write(_REG_CONFIG,self.config)

    def getShuntVoltage_mV(self):
        self.write(_REG_CALIBRATION,self._cal_value)
        value = self.read(_REG_SHUNTVOLTAGE)
        if value > 32767:
            value -= 65535
        return value * 0.01

    def getBusVoltage_V(self):
        self.write(_REG_CALIBRATION,self._cal_value)
        self.read(_REG_BUSVOLTAGE)
        return (self.read(_REG_BUSVOLTAGE) >> 3) * 0.004

    def getCurrent_mA(self):
        value = self.read(_REG_CURRENT)
        if value > 32767:
            value -= 65535
        return value * self._current_lsb

    def getPower_W(self):
        self.write(_REG_CALIBRATION,self._cal_value)
        value = self.read(_REG_POWER)
        if value > 32767:
            value -= 65535
        return value * self._power_lsb

class InterfaceState(Enum):
  DISABLED = 0
  ENABLED = 1
  CONNECTED = 2

# From my tests:
# over 4V => charging
# 4.7V => charging and charged 100%
# 3.9V => not charging, 100%
# 3.2V => will die in 10 mins under load, shut down
# 3.3V => warning icon?

# adc = Adafruit_ADS1x15.ADS1015()
# Choose a gain of 1 for reading voltages from 0 to 4.09V.
# Or pick a different gain to change the range of voltages that are read:
#  - 2/3 = +/-6.144V
#  -   1 = +/-4.096V
#  -   2 = +/-2.048V
#  -   4 = +/-1.024V
#  -   8 = +/-0.512V
#  -  16 = +/-0.256V
# See table 3 in the ADS1015i/ADS1115 datasheet for more info on gain.

ina219 = INA219(addr=0x42)

def translate_bat(voltage):
  global ina219

  if ina219.getCurrent_mA() > 0:
    state = "charging"
  else:
    state = "discharging"

  rightSpan = len(icons[state]) - 1

  valueScaled = (voltage - 6)/2.4

  # Convert the 0-1 range into a value in the right range.
  return icons[state][int(round(valueScaled * rightSpan))]

def wifi():
  global wifi_state, overlay_processes

  new_wifi_state = InterfaceState.DISABLED
  try:
    f = open(wifi_carrier, "r")
    carrier_state = int(f.read().rstrip())
    f.close()
    if carrier_state == 1:
      # ifup and connected to AP
      new_wifi_state = InterfaceState.CONNECTED
    elif carrier_state == 0:
      f = open(wifi_linkmode, "r")
      linkmode_state = int(f.read().rstrip())
      f.close()
      if linkmode_state == 1:
        # ifup but not connected to any network
        new_wifi_state = InterfaceState.ENABLED
        # else - must be ifdown
      
  except IOError:
    pass

  if new_wifi_state != wifi_state:
    if "wifi" in overlay_processes:
      overlay_processes["wifi"].kill()
      del overlay_processes["wifi"]

    if new_wifi_state == InterfaceState.ENABLED:
      overlay_processes["wifi"] = subprocess.Popen(pngview_call + [str(int(resolution[0]) - dpi * 2), wifi_icons["enabled"]])
    elif new_wifi_state == InterfaceState.DISABLED:
      overlay_processes["wifi"] = subprocess.Popen(pngview_call + [str(int(resolution[0]) - dpi * 2), wifi_icons["disabled"]])
    elif new_wifi_state == InterfaceState.CONNECTED:
      overlay_processes["wifi"] = subprocess.Popen(pngview_call + [str(int(resolution[0]) - dpi * 2), wifi_icons["connected"]])
  return new_wifi_state

def bluetooth():
  global bt_state, overlay_processes

  new_bt_state = InterfaceState.DISABLED
  try:
    p1 = subprocess.Popen('hciconfig', stdout = subprocess.PIPE)
    p2 = subprocess.Popen(['awk', 'FNR == 3 {print tolower($1)}'], stdin = p1.stdout, stdout=subprocess.PIPE)
    state=p2.communicate()[0].decode().rstrip()
    if state == "up":
      new_bt_state = InterfaceState.ENABLED
  except IOError:
    pass

  try:
    devices=os.listdir(bt_devices_dir)
    if len(devices) > 1:
      new_bt_state = InterfaceState.CONNECTED
  except OSError:
    pass

  if new_bt_state != bt_state:
    if "bt" in overlay_processes:
      overlay_processes["bt"].kill()
      del overlay_processes["bt"]

    if new_bt_state == InterfaceState.CONNECTED:
      overlay_processes["bt"] = subprocess.Popen(pngview_call + [str(int(resolution[0]) - dpi * 3), bt_icons["connected"]])
    elif new_bt_state == InterfaceState.ENABLED:
      overlay_processes["bt"] = subprocess.Popen(pngview_call + [str(int(resolution[0]) - dpi * 3), bt_icons["enabled"]])
    elif new_bt_state == InterfaceState.DISABLED:
      overlay_processes["bt"] = subprocess.Popen(pngview_call + [str(int(resolution[0]) - dpi * 3), bt_icons["disabled"]])
  return new_bt_state

def environment():
  global overlay_processes

  val=int(re.search("throttled=(0x\d+)", subprocess.check_output(env_cmd.split()).decode().rstrip()).groups()[0], 16)
  env = {
    "under-voltage": bool(val & 0x01),
    "freq-capped": bool(val & 0x02),
    "throttled": bool(val & 0x04)
  }
  for k,v in env.items():
    if v and not k in overlay_processes:
      overlay_processes[k] = subprocess.Popen(pngview_call + [str(int(resolution[0]) - dpi * (len(overlay_processes)+1)), env_icons[k]])
    elif not v and k in overlay_processes:
      overlay_processes[k].kill()
      del(overlay_processes[k])
  #return env # too much data
  return val

def battery():
  global battery_level, overlay_processes, battery_history, ina219
  value = ina219.getBusVoltage_V()

  battery_history.append(value)
  try:
    level_icon=translate_bat(median(battery_history))
  except IndexError:
    level_icon="unknown"


  if value <= 6.5:
    my_logger.warn("Battery voltage at or below 6.5V. Initiating shutdown within 30 seconds")

    subprocess.Popen(pngview_call + [str(int(resolution[0]) / 2 - 64), "-y", str(int(resolution[1]) / 2 - 64), icon_battery_critical_shutdown])
    os.system("sleep 30 && sudo poweroff &")

  if level_icon != battery_level:
    if "bat" in overlay_processes:
      overlay_processes["bat"].kill()
      del overlay_processes["bat"]

    icon='ic_battery_' + level_icon + "_white_" + str(dpi) + "dp.png"
    overlay_processes["bat"] = subprocess.Popen(pngview_call + [ str(int(resolution[0]) - dpi), iconpath + icon])
  return (level_icon, value)

overlay_processes = {}
wifi_state = None
bt_state = None
battery_level = None
env = None
battery_history = deque(maxlen=5)

# Set up logging
my_logger = logging.getLogger('MyLogger')
my_logger.setLevel(logging.INFO)
handler = logging.handlers.RotatingFileHandler(logfile, maxBytes=102400, backupCount=1)
my_logger.addHandler(handler)
console = logging.StreamHandler()
my_logger.addHandler(console)

# Get Framebuffer resolution
resolution=re.search("(\d{3,}x\d{3,})", subprocess.check_output(fbfile.split()).decode().rstrip()).group().split('x')
my_logger.info(resolution)

while True:
  (battery_level, value) = battery()
  wifi_state = wifi()
  bt_state = bluetooth()
  env = environment()
  my_logger.info("%s,median: %.2f, %s,icon: %s,wifi: %s,bt: %s, throttle: %#0x" % (
    datetime.now(),
    value,
    list(battery_history),
    battery_level,
    wifi_state.name,
    bt_state.name,
    env
  ))
  time.sleep(5)
