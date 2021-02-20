import socket


class TCP:
    def __init__(self, port,  server_socket=None):

        if server_socket is None:
            server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket = server_socket
        self.message = str
        self.port = port
        self.ip_address = socket.gethostbyname(socket.gethostname())
        self.server_address = (self.port, self.ip_address)
        self.client_socket = None

    def bind_server_socket(self):
        server_address = (self.ip_address, self.port)
        self.server_socket.bind(server_address) # binds to socket
        self.server_socket.listen(5) # listens for TCP connections
        print("[UNITY INFO] NOW LISTENING AT HOST ADDRESS: ", server_address)

    def listen_for_tcp(self):
        self.server_socket.listen(5)
        print("[INFO] NOW LISTENING AT: ", self.server_address)
        self.client_socket, client_address = self.server_socket.accept()
        print("[UNITY INFO] GOT CONNECTION FROM: ", client_address)

    def send_message(self, string):
        self.client_socket.send(bytes(string, "utf-8"))
