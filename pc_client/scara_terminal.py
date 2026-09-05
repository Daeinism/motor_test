"""Simple interactive terminal for controlling the SCARA robot over TCP."""

from __future__ import annotations

import argparse

from scara_client import ScaraClient


DEFAULT_HOST = "10.0.0.48"
DEFAULT_PORT = 5000


def print_received_message(message: str) -> None:
    print(f"\r{message}\nSCARA> ", end="", flush=True)


def print_disconnect_message(message: str) -> None:
    print(f"\n{message}")


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="SCARA TCP terminal")
    parser.add_argument("host", nargs="?", help="ESP32 IP address")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT)
    return parser.parse_args()


def main() -> None:
    arguments = parse_arguments()
    host = arguments.host

    if host is None:
        entered_host = input(f"ESP32 IP [{DEFAULT_HOST}]: ").strip()
        host = entered_host or DEFAULT_HOST

    client = ScaraClient(
        host,
        arguments.port,
        on_message=print_received_message,
        on_disconnect=print_disconnect_message,
    )

    try:
        print(f"Connecting to {host}:{arguments.port}...")
        client.connect()
        print("Connected. Type a SCARA command, or type exit to close the terminal.")

        while client.is_connected:
            command = input("SCARA> ").strip()
            if command.lower() in {"exit", "quit"}:
                break
            if command:
                client.send_command(command)
    except (ConnectionError, OSError) as error:
        print(f"Connection failed: {error}")
    except KeyboardInterrupt:
        print()
    finally:
        client.disconnect()
        print("SCARA terminal closed")


if __name__ == "__main__":
    main()
