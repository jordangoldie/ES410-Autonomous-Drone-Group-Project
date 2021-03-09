import socket


# a class to contain all tcp related objects, both server and client sockets
class TcpServer:

    # initialisations
    def __init__(self, port, name, server_socket=None):
        if server_socket is None:
            server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  # creates a server socket object
        self.server_socket = server_socket
        self.message = str
        self.port = port
        self.ip_address = socket.gethostbyname(socket.gethostname())
        self.server_address = (self.port, self.ip_address)
        self.client_socket = None
        self.name = name

    # binds server to socket
    def bind_server_socket(self):
        server_address = (self.ip_address, self.port)
        self.server_socket.bind(server_address) # binds to socket

    # listens for connections and creates client socket object instance
    def listen_for_tcp(self):
        self.server_socket.listen(5)
        print(f'[INFO {self.name}] NOW LISTENING AT: ', self.server_address)
        self.client_socket, client_address = self.server_socket.accept()
        print(f'[INFO {self.name}] GOT CONNECTION FROM: ', client_address)

    # encodes and sends a string over tcp
    def send_message(self, string):
        self.client_socket.send(bytes(string, "utf-8"))

    # decodes a received message, currently with a 1 byte buffer
    def receive_message(self):
        while True:
            msg = self.client_socket.recv(1)
            msg = msg.decode("utf-8")
            return msg



