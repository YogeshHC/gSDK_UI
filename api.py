from flask import Flask, request, jsonify, make_response
#from flask_sqlalchemy import SQLAlchemy
from werkzeug.security import generate_password_hash, check_password_hash
from werkzeug.utils import secure_filename
from werkzeug.datastructures import  FileStorage
import uuid
import jwt
import datetime
from functools import wraps
import os,json

app = Flask(__name__)
basepath = os.path.dirname(os.path.abspath(__file__))
dbpath = os.path.join(basepath,'sampledb.db')
events_upload_path = os.path.join(basepath,'Events')

# {"pitchUP":1,"pitchDown":-1,"yawRight":1,"yawLeft":-1,"Stop":0,"reset":0}
def updatestatus(key,value):
    configfile='params.json'
    data = {"pitchUP":0,"pitchDown":0,"yawRight":0,"yawLeft":0,"Stop":0,"reset":0}
    data[key]= value
    with open(configfile,'w') as f:
        json.dump(data,f)
    return True

@app.route('/panleft', methods=['GET', 'POST'])
def panleft():
	# import pdb;pdb.set_trace()
	formdata = request.get_json()
	moveto = formdata.get('moveto')
	print(moveto)
	updatestatus('yawLeft',-1)
	response = app.response_class(
			response=json.dumps({'status':'panleft success'}),
			status=200,
			mimetype='application/json'
		)
	return response

@app.route('/panright', methods=['GET', 'POST'])
def panright():
	formdata = request.get_json()
	moveto = formdata.get('moveto')
	print(moveto,'moveto')
	updatestatus('yawRight',1)
	response = app.response_class(
			response=json.dumps({'status':'panright success'}),
			status=200,
			mimetype='application/json'
		)
	return response

@app.route('/tiltup', methods=['GET', 'POST'])
def tiltup():
	formdata = request.get_json()
	moveto = formdata.get('moveto')
	updatestatus('pitchUP',1)
	response = app.response_class(
			response=json.dumps({'status':'tiltup success'}),
			status=200,
			mimetype='application/json'
		)
	return response

@app.route('/tiltdown', methods=['GET', 'POST'])
def tiltdown():
	formdata = request.get_json()
	moveto = formdata.get('moveto')
	updatestatus('pitchDown',-1)
	response = app.response_class(
			response=json.dumps({'status':'tiltdown success'}),
			status=200,
			mimetype='application/json'
		)
	return response

@app.route('/stop', methods=['GET', 'POST'])
def stop():
	formdata = request.get_json()
	moveto = formdata.get('moveto')
	updatestatus('Stop',0)
	response = app.response_class(
			response=json.dumps({'status':'stop success'}),
			status=200,
			mimetype='application/json'
		)
	return response


if __name__ == '__main__':  
	app.run(host='0.0.0.0', port=9000, debug=True)


# https://www.javacodemonk.com/part-2-deploy-flask-api-in-production-using-wsgi-gunicorn-with-nginx-reverse-proxy-4cbeffdb
