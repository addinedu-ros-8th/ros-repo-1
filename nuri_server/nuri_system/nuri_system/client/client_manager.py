import struct

class ClientManager:
    def __init__(self):
        self.clients = {}
        self.servers = {}
        self.devices = {}

    def register(self, handler, type=None, identifier=None):
        if type == "client":
            self.clients[handler.addr] = handler
        elif type == "server":
            self.servers[identifier] = handler
        elif type == "device":
            self.devices[identifier] = handler

    def unregister(self, handler):
        self.clients.pop(handler.addr, None)

        for key, h in list(self.servers.items()):
            if h == handler:
                self.servers.pop(key)

        for key, h in list(self.devices.items()):
            if h == handler:
                self.devices.pop(key)

    def get_client(self, addr):
        return self.clients.get(addr)

    def get_server(self, name):
        return self.servers.get(name)

    def get_device(self, device_id):
        return self.devices.get(device_id)
    
    def broadcast(self, data: bytes, target_group='client'):
        """
        target_group: 'client' | 'server' | 'device'
        """
        group = {
            'client': self.clients,
            'server': self.servers,
            'device': self.devices
        }.get(target_group, {})

        for key, handler in list(group.items()):
            try:
                # data += struct.pack('>H', len(key[0])) + key[0].encode('utf-8')
                handler.send(data)
            except Exception as e:
                print(f"[BROADCAST ERROR] {target_group.upper()} {key} → {e}")