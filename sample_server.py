# import socket
# serv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# serv.bind(('192.168.1.124', 8080))
# serv.listen(5)
# while True:
#     client, addr = serv.accept()    
#     print ('Got connection from', addr )
#     client.send('Thank you for connecting'.encode())
#     client.close()
#     break
import socket

sock = socket.socket()
print ("Socket created ...")

port = 9090
sock.bind(('', port))
sock.listen(5)

print ('socket is listening')

while True:
    c, addr = sock.accept()
    print ('got connection from ', addr)

    jsonReceived = c.recv(1024)
    
    print ("Json received -->", jsonReceived)

    c.close()