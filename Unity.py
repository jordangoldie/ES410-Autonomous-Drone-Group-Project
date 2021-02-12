


    def bind_server_socket(self, host_ip, port):
        host_name = socket.gethostname() # gets name of host machine
        host_ip = socket.gethostbyname(host_name) # gets IP of host machine
        port = 1234
        server_address = (host_ip, port)
        self.server_socket.bind(server_address) # binds to socket
        self.server_socket.listen(5) # listens for TCP connections
        print("[INFO] NOW LISTENING AT HOST ADDRESS: ", server_address)

    def listen_for_tcp(self):
        self.server_socket.listen(5)
        print("[INFO] NOW LISTENING AT: ", server_address)
        client_socket, client_address = self.server_socket.accept()
        print("[INFO] GOT CONNECTION FROM: ", client_address)