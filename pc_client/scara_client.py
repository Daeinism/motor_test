"""Reusable TCP client for the SCARA robot."""

from __future__ import annotations

import socket
import threading
from collections.abc import Callable


MessageHandler = Callable[[str], None]


class ScaraClient:
    """Maintains one TCP connection and exchanges newline-delimited messages."""

    def __init__(
        self,
        host: str,
        port: int = 5000,
        on_message: MessageHandler | None = None,
        on_disconnect: MessageHandler | None = None,
    ) -> None:
        self.host = host
        self.port = port
        self.on_message = on_message or (lambda message: None)
        self.on_disconnect = on_disconnect or (lambda message: None)
        self._socket: socket.socket | None = None
        self._connected = threading.Event()
        self._send_lock = threading.Lock()
        self._receiver_thread: threading.Thread | None = None

    @property
    def is_connected(self) -> bool:
        return self._connected.is_set()

    def connect(self, timeout: float = 5.0) -> None:
        if self.is_connected:
            return

        client_socket = socket.create_connection((self.host, self.port), timeout)
        client_socket.settimeout(None)
        self._socket = client_socket
        self._connected.set()
        self._receiver_thread = threading.Thread(
            target=self._receive_messages,
            name="scaraTcpReceiver",
            daemon=True,
        )
        self._receiver_thread.start()

    def send_command(self, command: str) -> None:
        command = command.strip()
        if not command:
            return
        if not self.is_connected or self._socket is None:
            raise ConnectionError("The SCARA TCP client is not connected")

        encoded_command = (command + "\n").encode("utf-8")
        with self._send_lock:
            self._socket.sendall(encoded_command)

    def disconnect(self) -> None:
        self._connected.clear()
        client_socket = self._socket
        self._socket = None

        if client_socket is not None:
            try:
                client_socket.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            client_socket.close()

    def _receive_messages(self) -> None:
        receive_buffer = ""
        disconnect_message = "Disconnected from the SCARA robot"

        try:
            while self.is_connected and self._socket is not None:
                received_data = self._socket.recv(1024)
                if not received_data:
                    break

                receive_buffer += received_data.decode("utf-8", errors="replace")
                while "\n" in receive_buffer:
                    message, receive_buffer = receive_buffer.split("\n", 1)
                    self.on_message(message.rstrip("\r"))
        except OSError as error:
            if self.is_connected:
                disconnect_message = f"TCP connection error: {error}"
        finally:
            was_connected = self.is_connected
            self.disconnect()
            if was_connected:
                self.on_disconnect(disconnect_message)
