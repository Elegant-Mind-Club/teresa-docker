import zmq
import time


def main():
    # ZMQ Context
    context = zmq.Context()

    # Create a PULL socket to receive messages
    socket = context.socket(zmq.PULL)

    # Bind the socket to all interfaces on port 10321
    socket.bind("tcp://*:10321")

    print(f"[HOST] Listening on port 10321...")

    try:
        while True:
            # Receive message (non-blocking)
            try:
                message = socket.recv_string(flags=zmq.NOBLOCK)
                print(f"[HOST] Received: {message}")
            except zmq.Again:
                # No message available
                time.sleep(0.001)  # Small sleep to prevent CPU spinning
                continue

    except KeyboardInterrupt:
        print("\n[HOST] Shutting down...")
    finally:
        # Clean up
        socket.close()
        context.term()


if __name__ == "__main__":
    main()
