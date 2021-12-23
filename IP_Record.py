import socket	#for sockets
import sys	#for exit

# create dgram udp socket
try:
	s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
except socket.error:
	print('Failed to create socket')
	sys.exit()

host = '192.168.19.1';
port = 3006;

while(1) :
	
	try :
		
		# receive data from client (data, addr)
		d = s.recvfrom(1024)
		reply = d[0]
		addr = d[1]
		
		print('Server reply : ' + reply)
	
	except socket.error:
		print('Error Code : ')
		sys.exit()
