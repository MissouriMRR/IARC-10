# Server is just writing data to a file

# May want to look into using udp instead
import socket


class Server:
    # Server Class constructor. Used to pass in JSON Data
    def __init__(self, jsonData):
        self.jsonData = jsonData
        self.droneId: str = jsonData["localInfo"]["selfId"]

    # Starts functionality of server
    # TODO separate into 2 functions for server setup and server runtime
    def start_server(self):
        # Init server socket
        server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        # TODO Update to take specific drone value (EX: Drone 2)
        server_socket.bind(
            (
                str(self.jsonData["drones"][self.droneId]["ip"]),
                int(self.jsonData["drones"][self.droneId]["port"]),
            )
        )
        server_socket.listen(5)

        try:
            while True:
                # Wait for socket connection and take client socket and address variables
                client_socket, client_address = server_socket.accept()
                print(f"Connection accepted from {client_address}")

                # Get data from client socket
                data = client_socket.recv(1024)
                print(f"Received from client: {data.decode()}")

                # Respond to client message to confirm transmission
                response = "Server received your message!"
                client_socket.sendall(response.encode())

                # Close connection to client socket
                client_socket.close()
        except KeyboardInterrupt:
            print("Server shutting down.")
        finally:
            # When program is killed, close server socket
            server_socket.close()
