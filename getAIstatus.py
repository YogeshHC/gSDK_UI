import time
import sys
import subprocess
import threading
import queue
import json
# deepstream-app -c /opt/nvidia/deepstream/deepstream-5.0/samples/configs/deepstream-app/custom_config_v1.txt
# deepstream-app -c /opt/nvidia/deepstream/deepstream-5.0/samples/configs/deepstream-app/custom_config_v1.txt

cmd='deepstream-app -c /opt/nvidia/deepstream/deepstream-5.0/samples/configs/deepstream-app/custom_config_v1.txt'
# cmd='ping -t 192.168.1.231'

def getdata(key):
    configfile='/home/agx2/v4_gSDK_UI_latest/devicestatus.json'
    with open(configfile,'r') as f:
        try:
            data = json.load(f)
            if data[key] == 'active':
                return True
        except:
            return False
    return False

class ExecutorFlushSTDOUT(object):
    def __init__(self,timeout=20):
        self.process = None
        self.process_output = queue.Queue(0)
        self.capture_output = threading.Thread(target=self.output_reader,daemon=True)
        self.timeout=timeout
        self.result=False
        self.validator=None
        
    def output_reader(self):
        start=time.time()
        while self.process.poll() is None and (time.time() - start) < self.timeout:
            try:
                if not self.process_output.full():
                    line=self.process.stdout.readline()
                    if line:
                        line=line.decode().rstrip("\n")
                        start=time.time()
                        self.process_output.put(line)
                        if self.validator:
                            if self.validator in line: print("Valid");self.result=True

            except:pass
        self.process.kill()
        return
            
    def start_process(self,cmd_list,callback=None,validator=None,timeout=None):
        if timeout: self.timeout=timeout
        self.validator=validator
        self.process = subprocess.Popen(cmd_list,stdout=subprocess.PIPE,stderr=subprocess.PIPE,shell=True)
        self.capture_output.start()
        line=None
        self.result=False
        while self.process.poll() is None:
            try:
                if not self.process_output.empty():
                    line = self.process_output.get()
                if line:
                    if callback:callback(line)
                    line=None
            except Exception as e:print('exception--------',e)      
        error = self.process.returncode
        if error:
            print("Error Found",str(error))
            raise RuntimeError(error)
        return self.result

execute = ExecutorFlushSTDOUT()
def liveOUTPUT(line):
    # print('log->',line)
    if line.startswith("**PERF:  ") and 'FPS' not in line:
        status = 'inactive'
        print(line)
    if 'Resetting source 0' in line or "**PERF:  0.00 (0.00)" in line:
        status = 'inactive'
    else:
        status = 'active'
    # print(status)
    with open('params.json', 'r') as file:
        response = json.load(file)
    response['ai'] = status
    with open('params.json', 'w') as file:
        json.dump(response, file)

print('----------************----------')
camerastatus = getdata('camera')
while not camerastatus:
    camerastatus = getdata('camera')
    print('camera status -->',camerastatus)
result=execute.start_process(cmd,callback=liveOUTPUT,validator="Hash of data verified.")
print("Finish",result)
