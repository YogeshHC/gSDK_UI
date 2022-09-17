import socket
import json
from pprint import pprint

sock = socket.socket()
print ("Socket created ...")

port = 9090
sock.bind(('', port))
sock.listen(5)

print ('socket is listening')

def getdefaultvalues():
    posmap_default = {}
    gimbal_controls_reset = {"Stop" : 0,"pitchDown" : 0, "pitchUP" : 0, "yawLeft" : 0, "yawRight" : 0}
    sweepcontrols = {"Speed" : 5,"Sweep" : 0,"angle" : 10}
    gimbalstatus = {"error" : 0,"ready" : 1,"reset" : 0}
    gimbalpos = {"pitch":0.00,"roll":0.00,"yaw":0.00}
    posmap_default['GimbalControls'] = gimbal_controls_reset
    posmap_default['SweepControls'] = sweepcontrols
    posmap_default['GimbalStatus'] = gimbalstatus
    posmap_default['GimbalPos'] = gimbalpos
    posmap_default['Reboot'] = 0
    posmap_default["GetPos"] = 0
    posmap_default["SetPos"] = 0
    return posmap_default

while True:
    posmap_default = getdefaultvalues()
    socketconn, addr = sock.accept()
    print ('Received Data from ', addr)
    jsonReceived = socketconn.recv(1024).decode()
    print(jsonReceived,'->->->')
    if jsonReceived:
        operation_obj = json.loads(jsonReceived)
        print('JSON Received ',jsonReceived)
        # update operations
        if operation_obj["key"] != 'devicestatus':
            if operation_obj["key"]=='SweepControls' and operation_obj["operation"] != "defaultparams":
                with open('params.json', 'r') as file:
                    response = json.load(file)
                    gimbal_sweep_data = response['SweepControls']
                gimbal_sweep_data[operation_obj["operation"]] = operation_obj["operationval"]
                posmap_default["SweepControls"] = gimbal_sweep_data
            elif operation_obj["operation"] not in ['GetPos','SetPos'] and operation_obj["operation"] !="defaultparams":
                posmap_default[operation_obj["key"]][operation_obj["operation"]] = operation_obj["operationval"]
            elif operation_obj["operation"] in "GetPos":
                posmap_default['GetPos']=1
            elif operation_obj["operation"] in "SetPos":
                posmap_default[operation_obj["key"]] = operation_obj["position"]
                posmap_default['SetPos']=1

            with open('params.json', 'w') as file:
                json.dump(posmap_default, file)
            pprint(posmap_default)
            # get operations
            if operation_obj["operation"] == 'GetPos':
                with open('params.json', 'r') as file:
                    response = json.load(file)
                    gimbal_pos_data = response['GimbalPos']
                    response_data = json.dumps(gimbal_pos_data)
                    response_data = response_data.encode()
                socketconn.sendall(response_data)
                posmap_default['GetPos']=0
                with open('params.json', 'w') as file:
                    json.dump(posmap_default, file)
            elif operation_obj["key"] == 'SweepControls' and operation_obj["operation"]=="defaultparams":
                print('in default params')
                with open('params.json', 'r') as file:
                    response = json.load(file)
                    gimbal_pos_data = response['SweepControls']
                    response_data = json.dumps(gimbal_pos_data)
                    response_data = response_data.encode()
                socketconn.sendall(response_data)
            else:
                response = {"status":"success","operation":operation_obj["operation"]}
                response_data = json.dumps(response)
                socketconn.sendall(response_data.encode())
        else:
            with open('devicestatus.json', 'r') as file:
                response = json.load(file)
                response_data = json.dumps(response)
                response_data = response_data.encode()
            socketconn.sendall(response_data)
    if not jsonReceived: break

    socketconn.close()
