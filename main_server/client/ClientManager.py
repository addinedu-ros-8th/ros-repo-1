class ClientManager:
    def __init__(self):
        self.clients = set()

    def register(self, handler):
        self.clients.add(handler)

    def unregister(self, handler):
        self.clients.discard(handler)

    def broadcast(self, data: bytes):
        for client in list(self.clients):
            try:
                client.send_response(data)
            except Exception as e:
                print(f"[BROADCAST ERROR] {client.addr} → {e}")