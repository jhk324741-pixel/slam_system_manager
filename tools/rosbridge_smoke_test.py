#!/usr/bin/env python3
"""Read-only rosbridge smoke test for slam_system_manager."""

import argparse
import json
import sys
import time
import uuid

try:
    import websocket
except ImportError as exception:
    raise SystemExit(
        "websocket-client is required; install Ubuntu package python3-websocket"
    ) from exception


STATUS_FIELDS = {
    "state",
    "ouster_running",
    "pointcloud_alive",
    "imu_alive",
    "pointcloud_hz",
    "imu_hz",
    "mapping_running",
    "localization_running",
    "current_map",
    "error_code",
    "error_message",
}
BOOLEAN_FIELDS = {
    "ouster_running",
    "pointcloud_alive",
    "imu_alive",
    "mapping_running",
    "localization_running",
}
STRING_FIELDS = {"state", "current_map", "error_code", "error_message"}
COMPARISON_FIELDS = STATUS_FIELDS - {"pointcloud_hz", "imu_hz"}


class SmokeTestError(RuntimeError):
    """Raised when rosbridge does not satisfy the Phase 8 contract."""


class RosbridgeClient:
    def __init__(self, url, timeout):
        self.url = url
        self.timeout = timeout
        self.socket = None

    def connect(self):
        self.socket = websocket.create_connection(self.url, timeout=self.timeout)

    def close(self):
        if self.socket is not None:
            self.socket.close()
            self.socket = None

    def send(self, payload):
        self.socket.send(json.dumps(payload, separators=(",", ":")))

    def receive_until(self, predicate, description):
        deadline = time.monotonic() + self.timeout
        while time.monotonic() < deadline:
            self.socket.settimeout(max(0.1, deadline - time.monotonic()))
            try:
                raw_message = self.socket.recv()
            except websocket.WebSocketTimeoutException:
                continue
            if isinstance(raw_message, bytes):
                raw_message = raw_message.decode("utf-8")
            try:
                message = json.loads(raw_message)
            except json.JSONDecodeError:
                continue
            if predicate(message):
                return message
        raise SmokeTestError(f"Timed out waiting for {description}")

    def subscribe_status(self):
        request_id = f"status-{uuid.uuid4()}"
        self.send(
            {
                "op": "subscribe",
                "id": request_id,
                "topic": "/system/status",
                "type": "slam_system_manager/msg/SystemStatus",
            }
        )
        message = self.receive_until(
            lambda item: item.get("op") == "publish"
            and item.get("topic") == "/system/status",
            "/system/status publication",
        )
        return message.get("msg")

    def call_service(self, service, args=None):
        request_id = f"service-{uuid.uuid4()}"
        self.send(
            {
                "op": "call_service",
                "id": request_id,
                "service": service,
                "args": args or {},
            }
        )
        response = self.receive_until(
            lambda item: item.get("op") == "service_response"
            and item.get("id") == request_id,
            f"response from {service}",
        )
        if response.get("result") is False:
            raise SmokeTestError(
                f"rosbridge rejected {service}: {response.get('values', response)}"
            )
        values = response.get("values")
        if not isinstance(values, dict):
            raise SmokeTestError(f"{service} returned no values object")
        return values


def validate_status(status, source):
    if not isinstance(status, dict):
        raise SmokeTestError(f"{source} status is not a JSON object")
    missing = sorted(STATUS_FIELDS - status.keys())
    if missing:
        raise SmokeTestError(f"{source} status is missing fields: {', '.join(missing)}")
    for field in BOOLEAN_FIELDS:
        if type(status[field]) is not bool:
            raise SmokeTestError(f"{source}.{field} is not a boolean")
    for field in STRING_FIELDS:
        if not isinstance(status[field], str):
            raise SmokeTestError(f"{source}.{field} is not a string")
    for field in ("pointcloud_hz", "imu_hz"):
        if not isinstance(status[field], (int, float)) or isinstance(status[field], bool):
            raise SmokeTestError(f"{source}.{field} is not numeric")


def run_smoke_test(url, timeout):
    client = RosbridgeClient(url, timeout)
    try:
        print(f"[1/6] Connecting to {url}")
        client.connect()

        print("[2/6] Subscribing to /system/status")
        topic_status = client.subscribe_status()
        validate_status(topic_status, "topic")

        print("[3/6] Calling /system/get_status")
        status_response = client.call_service("/system/get_status")
        if status_response.get("success") is not True:
            raise SmokeTestError(f"get_status failed: {status_response}")
        service_status = status_response.get("status")
        validate_status(service_status, "service")
        mismatches = [
            field
            for field in sorted(COMPARISON_FIELDS)
            if topic_status[field] != service_status[field]
        ]
        if mismatches:
            raise SmokeTestError(
                "Topic/service status mismatch: " + ", ".join(mismatches)
            )

        print("[4/6] Calling /system/get_map_list")
        map_response = client.call_service("/system/get_map_list")
        if map_response.get("success") is not True:
            raise SmokeTestError(f"get_map_list failed: {map_response}")
        if not isinstance(map_response.get("map_names"), list):
            raise SmokeTestError("get_map_list.map_names is not an array")

        print("[5/6] Disconnecting and reconnecting")
        client.close()
        client.connect()

        print("[6/6] Confirming status after reconnect")
        reconnect_status = client.subscribe_status()
        validate_status(reconnect_status, "reconnect")
    finally:
        client.close()

    print("PASS: rosbridge Phase 8 read-only smoke test completed")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--url", default="ws://127.0.0.1:9090")
    parser.add_argument("--timeout", type=float, default=10.0)
    arguments = parser.parse_args()
    if arguments.timeout <= 0.0:
        parser.error("--timeout must be greater than zero")

    try:
        run_smoke_test(arguments.url, arguments.timeout)
    except (OSError, websocket.WebSocketException, SmokeTestError) as exception:
        print(f"FAIL: {exception}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
