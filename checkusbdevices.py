import re,os,time,json
import subprocess,multiprocessing

def resetdata():
    configfile='/home/agx/person_segmentation/config.json'
    data = {"serialstatus":"inactive","joystickstatus":"inactive","camerastatus":"inactive","gimbalcontrol":"inactive","aistatus":"inactive"}
    with open(configfile,'w') as f:
        json.dump(data,f)
    return True

def updatestatus(key,value):
    configfile='/home/agx/person_segmentation/config.json'
    data = {"serialstatus":"inactive","joystickstatus":"inactive","camerastatus":"inactive","gimbalcontrol":"inactive","aistatus":"inactive"}
    with open(configfile,'r') as f:
        try:
            data = json.load(f)
            data[key] = value
        except:
            data[key]= value

    with open(configfile,'w') as f:
        json.dump(data,f)
    return True

def chcekUSBserialdevice():
    return os.path.exists('/dev/ttyUSB0')

def checkcontroller():
    device_re = re.compile("Bus\s+(?P<bus>\d+)\s+Device\s+(?P<device>\d+).+ID\s(?P<id>\w+:\w+)\s(?P<tag>.+)$", re.I)
    df = subprocess.check_output("lsusb").decode("utf-8")
    devices = []
    known_devices = ['Prolific Technology, Inc. PL2303 Serial Port','Microsoft Corp. Xbox360 Controller']
    for i in df.split('\n'):
        if i:
            info = device_re.match(i)
            dinfo = info.groupdict()
            if info['tag'] in known_devices:
                dinfo['device'] = '/dev/bus/usb/%s/%s' % (dinfo.pop('bus'), dinfo.pop('device'))
                devices.append(dinfo)
    if len(devices)==2:
        return True
    else:
        return False

def getcameraip(macaddress):
    mactofind = f'MAC Address: {macaddress} (Z3 Technology)'
    data_map = []
    ip_re = r'[0-9]+(?:\.[0-9]+){3}'
    mac_re = r'(?:[0-9a-fA-F]:?){12}'
    cmd_out = os.popen("nmap --privileged -sn 192.168.1.0/24").read()
    line_arr = cmd_out.split('\n')
    line_arr = line_arr[2:-4]
    if mactofind in line_arr:
        result = re.findall(ip_re,line_arr[line_arr.index(mactofind)-2])[0]
        return {'status':True,'ip':result}
    return {'status':False,'ip':''}
    
def startgimbaloperation():
    #print('camera-ip',cameraip)
    gimbalprocess = subprocess.Popen(['./gSDK'], stdout=subprocess.PIPE,shell=True, stderr=subprocess.STDOUT)
    with gimbalprocess.stdout:
        for line in iter(gimbalprocess.stdout.readline, b''):
            print(line.decode("utf-8").strip())
    # gimbalprocess.start()

resetdata()
Serial_status = chcekUSBserialdevice()
data = {"serialstatus":"inactive","joystickstatus":"inactive","camerastatus":"inactive","gimbalcontrol":"inactive","aistatus":"inactive"}
while not Serial_status:
    Serial_status = chcekUSBserialdevice()
    time.sleep(1)
updatestatus(key="serialstatus",value="active")
print('------- USB device is connected -------')
#Controller_status = checkcontroller()
#while not Controller_status:
#    Controller_status = checkcontroller()
#    time.sleep(1)
#print('------- USB Controller is connected -------')
#updatestatus(key="joystickstatus",value="active")
#camera_status = getcameraip('40:CD:3A:03:25:38')
#while not camera_status['status']:
#    camera_status = getcameraip('40:CD:3A:03:25:38')
#    time.sleep(1)
#updatestatus(key="camerastatus",value="active")
#print('------- Camera is online -------')
gimbal_status = startgimbaloperation
gimbal_status()
updatestatus(key="gimbalcontrol",value="active")
print('------ All services are active ------')
