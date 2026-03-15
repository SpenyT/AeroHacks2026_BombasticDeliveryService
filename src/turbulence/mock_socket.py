class MockSocket:
    def __init__(self):
        self.buffer = b""

    def settimeout(self, timeout):
        pass

    def connect(self, addr):
        print(f"[mock] connected to {addr}")

    def sendall(self, data):
        self.buffer += self._fake_response(data)

    def recv(self, n):
        if not self.buffer:
            return b""

        chunk = self.buffer[:n]
        self.buffer = self.buffer[n:]
        return chunk


    def _fake_response(self, data):
        cmd = data.decode().strip()

        if cmd == "gMode":
            return b"2\n"

        if cmd == "angX":
            return b"32\n"

        if cmd == "angY":
            return b"-16\n"

        if cmd == "gyroX":
            return b"0.2\n"

        if cmd == "gyroY":
            return b"-0.1\n"

        if cmd == "geti":
            return b"0.01,-0.02\n"

        return b"ok\n"
