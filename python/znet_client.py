import sys
import socket
import time
import threading
from datetime import datetime
import random
import ctypes


HOST = 'localhost'
PORT = 50077
router01=0
router02=0
coordinator=0
terminal=["AA01","AA02","AA03","AA04"]  #终端短地址列表
recv_flag=1
send_flag=1
fix_comm_data = '6E01A213B43FC49C71'
fix_coor_command = "FC070303"   #协调器发送指令
data_file = "~/znet.log"
SEND_INTERVAL_MIN = 12
SEND_INTERVAL_MAX = 18
current_terminal = ""
sending_flag = 0

s=socket.socket(socket.AF_INET,socket.SOCK_STREAM)

class listen_terminal_back(threading.Thread):
    def __init__(self, *param):
        threading.Thread.__init__(self)
        self.name, self.sock = param

    def run(self):
        try:
            while True:
                recv_buf = self.sock.recv(128)
                print(str(recv_buf))
                #recv_data = recv_buf.split('+')
        finally:
            print('end')

    def get_id(self):

        # returns id of the respective thread
        if hasattr(self, '_thread_id'):
            return self._thread_id
        for id, thread in threading._active.items():
            if thread is self:
                return id

    def raise_exception(self):
        thread_id = self.get_id()
        res = ctypes.pythonapi.PyThreadState_SetAsyncExc(thread_id,
            ctypes.py_object(SystemExit))
        if res > 1:
            ctypes.pythonapi.PyThreadState_SetAsyncExc(thread_id, 0)
            print('Exception raise failure')


#接收信息线程处理函数
def response_recv():
    while recv_flag:
        recv_buf = s.recv(128).decode('utf-8')
        recv_data = recv_buf.split('+')
       # if recv_data[:1] != fix_comm_data:
       #     with open(data_file, 'w', encoding='utf-8') as log:
       #         log.write("data transfer error:  " + current_terminal)
        print(recv_buf, '\n')
    print("response quit")


#发送线程处理函数
def send_tmsg():
    global current_terminal
    terminal_index = 0
    while send_flag:
        send_time = datetime.strftime(datetime.now(),"%m-%d %H:%M:%S:%f")
        current_terminal = terminal[terminal_index]
        s.sendall((fix_coor_command + current_terminal + '+' + send_time + '+' + fix_comm_data).encode("utf-8"))
        terminal_index = 0 if terminal_index == 3 else terminal_index + 1
        time.sleep(random.randint(SEND_INTERVAL_MIN,SEND_INTERVAL_MAX))
    print("send quit")

if __name__ == '__main__':
    try:
        s.connect((HOST,PORT))
    except socket.error as e:
        print("Connection error: %s" % e)
        sys.exit(1)
    #s.setblocking(False)
    #r = threading.Thread(target=response_recv)
    sn = threading.Thread(target=send_tmsg)
    thread_test = listen_terminal_back("LTB", s)
    thread_test.daemon = True
    thread_test.start()
    sn.daemon = True
    sn.start()
   # r.daemon = True
   # r.start()
    time.sleep(60)
    print("main is quitting")
    thread_test.raise_exception()
    thread_test.join()
    recv_flag = 0
    send_flag = 0
    #r.join()
    sn.join()
    s.close()
