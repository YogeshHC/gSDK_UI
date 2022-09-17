import re,os,time,json
import subprocess,multiprocessing

def resetdata():
    configfile='/home/agx2/v4_gSDK_UI_latest/devicestatus.json'
    data = {"serialconn":"inactive","gimbal":"inactive","ai":"inactive","camera":"inactive"}
    with open(configfile,'w') as f:
        json.dump(data,f)
    return True

def updatestatus(key,value):
    configfile='/home/agx2/v4_gSDK_UI_latest/devicestatus.json'
    data = {"serialconn":"inactive","gimbal":"inactive","ai":"inactive","camera":"inactive"}
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
    if os.path.exists('/dev/ttyUSB0'):
        return 'active'
    else:
        return  'inactive'

def startgimbaloperation():
    gimbalprocess = subprocess.Popen(['./gSDK'], stdout=subprocess.PIPE,shell=True, stderr=subprocess.STDOUT)
    with gimbalprocess.stdout:
        for line in iter(gimbalprocess.stdout.readline, b''):
            print(line.decode("utf-8").strip())

resetdata()
Serial_status = chcekUSBserialdevice()
data = {"serialconn":"inactive","gimbal":"inactive","ai":"inactive"}
while True:
    Serial_status = chcekUSBserialdevice()
    updatestatus(key="serialconn",value=Serial_status)
    print('------- USB device status -------',Serial_status)
    time.sleep(2)




gimbal_status = startgimbaloperation
gimbal_status()

updatestatus(key="gimbalcontrol",value="active")
print('------ All services are active ------')
