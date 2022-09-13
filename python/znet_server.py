from gettext import bind_textdomain_codeset
import socket
import sys

s=socket.socket(socket.AF_INET,socket.SOCK_STREAM)

host = socket.socket.gethostname()
port = 12345
s.bind((host, port))
s.listen(5)
while True:
    c, addr = s.accept()
    print ("client connect in: ", addr)
    c.sent("welcome!")