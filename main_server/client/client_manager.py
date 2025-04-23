class ClientManager:
    def __init__(self):
        self.clients = {}

    def register(self, handler, server=None):
        if server is not None:
            self.clients[server] = handler
        else:
            self.clients[handler.addr] = handler

    def unregister(self, handler):
        self.clients.pop(handler.addr, None)

    def get(self, addr):
        return self.clients.get(addr)

    def broadcast(self, data: bytes):
        for addr, client in list(self.clients.items()):
            try:
                client.send(data)
            except Exception as e:
                print(f"[BROADCAST ERROR] {addr} → {e}")