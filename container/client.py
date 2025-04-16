import zmq
import time

HOST = "host.docker.internal"
PORT = 10321

print("[CONTAINER] Starting up...")
context = zmq.Context()
socket = context.socket(zmq.PUSH)
socket.connect(f"tcp://{HOST}:{PORT}")
print(f"[CONTAINER] Connected to host at {HOST}:{PORT}")

for i in range(100):
    msg = f"Message {i} from Ubuntu container!dlkjfsdkljflksdjflkdsjdslkdlkjfsdkljflksdjflkdsjdslkdlkjfsdkljflksdjflkdsjdslkdlkjfsdkljflksdjflkdsjdslkdlkjfsdkljflksdjflkdsjdslkdlkjfsdkljflksdjflkdsjdslkdlkjfsdkljflksdjflkdsjdslkdlkjfsdkljflksdjflkdsjdslkdlkjfsdkljflksdjflkdsjdslkdlkjfsdkljflksdjflkdsjdslkdlkjfsdkljflksdjflkdsjdslkdlkjfsdkljflksdjflkdsjdslk"
    socket.send_string(msg)
    print(f"[CONTAINER] Sent: {msg}")
