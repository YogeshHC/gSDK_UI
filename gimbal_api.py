from flask import Flask, request, jsonify, make_response
from werkzeug.security import generate_password_hash, check_password_hash
from werkzeug.utils import secure_filename
from werkzeug.datastructures import  FileStorage
from functools import wraps
import os
import subprocess

class Datastorage(object):

	def __init__(self):
		self.availabledata = {}
		pass

	def setdata(self, key, value):
		self.availabledata[key] = value
		return self.availabledata

	def getdata(self, key):
		return self.availabledata.get(key,'')

app = Flask(__name__)
basepath = os.path.dirname(os.path.abspath(__file__))	
db = Datastorage()
	
@app.route('/startgimbal', methods=['GET', 'POST'])
def startgimbal():
	print('starting gimbal')
	gimbalsubprocess = subprocess.Popen(['./gSDK'], shell=True)
	savedata = db.setdata('gimbalprocessobj',gimbalsubprocess)
	if 'gimbalprocessobj' in savedata:
		return f'Gimbal setup is started with process id - {gimbalsubprocess.pid}'
	else:
		return f'Unable to start gimbal'

@app.route('/terminate', methods=['GET', 'POST'])
def terminate():
	print('terminating gimbal')
	if db.getdata('gimbalprocessobj') != '':
		pid = db.getdata('gimbalprocessobj').pid
		db.getdata('gimbalprocessobj').kill()
		return f'gimbal process terminated with pid - {pid}'
	else:
		return 'gimbal process is not found'

if __name__ == '__main__':  
	app.run(host='0.0.0.0', port=9090, debug=True)

