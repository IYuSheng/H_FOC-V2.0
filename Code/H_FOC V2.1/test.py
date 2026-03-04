"""
CAN broadcast receive test for the current firmware FDCAN settings.

Firmware-matched settings:
- Frame: Classic CAN
- ID type: Standard 11-bit
- Nominal bitrate: 1 Mbps
- ID layout: (src << 8) | (dst << 4) | type
"""

from __future__ import annotations

import argparse
import struct
import time
from typing import Dict, Optional, Tuple

try:
    import can
except ImportError as exc:  # pragma: no cover
    raise SystemExit(
        "python-can is required. Install with: pip install python-can"
    ) from exc


# Keep in sync with Core/Inc/fdcan.h
JOINT_ID_LOCAL = 0x02
JOINT_ID_BROADCAST = 0x0F
MSG_TYPE_STATUS = 0x0

SCALE_32 = 0.00001
STATUS_PAYLOAD_LEN = 8  # sizeof(joint_status_t)


def parse_can_id(can_id: int) -> Tuple[int, int, int]:
    """Parse firmware-defined CAN ID fields."""
    src = (can_id >> 8) & 0x0F
    dst = (can_id >> 4) & 0x0F
    msg_type = can_id & 0x0F
    return src, dst, msg_type


def decode_joint_status(payload: bytes) -> Optional[Dict[str, float]]:
    """
    Decode joint_status_t payload from firmware:
    int32 position, int32 velocity.
    """
    if len(payload) < STATUS_PAYLOAD_LEN:
        return None

    pos_raw, vel_raw = struct.unpack("<ii", payload[:STATUS_PAYLOAD_LEN])
    return {
        "position": pos_raw * SCALE_32,
        "velocity": vel_raw * SCALE_32,
    }


def test_receive_broadcast(
    interface: str,
    channel: str,
    receive_seconds: float = 10.0,
    recv_timeout: float = 0.2,
    nominal_bitrate: int = 1_000_000,
    expected_src: Optional[int] = JOINT_ID_LOCAL,
) -> bool:
    """
    Validate whether motor broadcast status frames are received correctly.

    Returns:
        True if at least one valid broadcast status frame is received,
        otherwise False.
    """
    bus_kwargs = {
        "interface": interface,
        "channel": channel,
        "bitrate": nominal_bitrate,
    }

    try:
        bus = can.Bus(**bus_kwargs)
    except TypeError:
        # Some adapters/channels do not accept bitrate arguments from python-can
        bus_kwargs.pop("bitrate")
        bus = can.Bus(**bus_kwargs)
    except Exception as exc:
        raise SystemExit(
            "Failed to open CAN bus with "
            f"interface={interface}, channel={channel}. "
            "Check that the port is correct and not occupied by another software. "
            f"Original error: {exc}"
        ) from exc

    valid_count = 0
    valid_from_expected_src = 0
    format_mismatch_count = 0
    payload_mismatch_count = 0
    total_rx_count = 0

    print(
        f"[INFO] Listening on interface={interface}, channel={channel}, "
        f"classic_can=True, bitrate={nominal_bitrate}"
    )
    print(
        f"[INFO] Waiting {receive_seconds:.1f}s for broadcast frames "
        f"(dst=0x{JOINT_ID_BROADCAST:X}, type=0x{MSG_TYPE_STATUS:X})..."
    )
    if expected_src is not None:
        print(f"[INFO] Expecting source node id: {expected_src}")

    start = time.monotonic()
    try:
        while (time.monotonic() - start) < receive_seconds:
            msg = bus.recv(timeout=recv_timeout)
            if msg is None:
                continue

            total_rx_count += 1
            if msg.is_remote_frame:
                continue

            src, dst, msg_type = parse_can_id(msg.arbitration_id)
            if dst != JOINT_ID_BROADCAST or msg_type != MSG_TYPE_STATUS:
                continue

            # Match firmware send format: Standard ID + Classic CAN
            if msg.is_extended_id or msg.is_fd:
                format_mismatch_count += 1
                print(
                    "[WARN] Broadcast status frame received but format mismatch: "
                    f"std={not msg.is_extended_id}, fd={msg.is_fd}, "
                    f"id=0x{msg.arbitration_id:03X}, dlc={msg.dlc}"
                )
                continue

            payload = bytes(msg.data)
            if len(payload) != STATUS_PAYLOAD_LEN:
                payload_mismatch_count += 1
                print(
                    "[WARN] Broadcast status payload length mismatch: "
                    f"expected={STATUS_PAYLOAD_LEN}, actual={len(payload)}, "
                    f"id=0x{msg.arbitration_id:03X}"
                )
                continue

            decoded = decode_joint_status(payload)

            print(
                "[RX] "
                f"id=0x{msg.arbitration_id:03X} src={src} dst={dst} type={msg_type} "
                f"len={len(payload)} data={payload.hex(' ')}"
            )
            if decoded is not None:
                print(
                    "[RX] decoded: "
                    f"pos={decoded['position']:.5f}, vel={decoded['velocity']:.5f}"
                )

            valid_count += 1
            if expected_src is None or src == expected_src:
                valid_from_expected_src += 1
    finally:
        bus.shutdown()

    print(
        "[RESULT] "
        f"total_rx={total_rx_count}, valid_broadcast_status={valid_count}, "
        f"valid_from_expected_src={valid_from_expected_src}, "
        f"format_mismatch={format_mismatch_count}, payload_mismatch={payload_mismatch_count}"
    )

    if valid_from_expected_src > 0:
        print("[PASS] Broadcast receive is working with current CAN configuration.")
        return True

    print("[FAIL] No valid broadcast status frame received from expected source in time window.")
    return False


def _build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Test receiving motor CAN broadcast frames (firmware-matched settings)."
    )
    parser.add_argument("--interface", help="python-can interface, e.g. pcan/vector/socketcan")
    parser.add_argument("--channel", help="Channel name, e.g. PCAN_USBBUS1/can0")
    parser.add_argument("--seconds", type=float, default=10.0, help="Listen duration in seconds")
    parser.add_argument("--recv-timeout", type=float, default=0.2, help="Per-recv timeout in seconds")
    parser.add_argument("--bitrate", type=int, default=1_000_000, help="Nominal bitrate")
    parser.add_argument(
        "--expected-src",
        type=int,
        default=JOINT_ID_LOCAL,
        help="Expected source node ID (0-14). Use -1 to accept any source.",
    )
    parser.add_argument(
        "--list",
        action="store_true",
        help="List detected CAN configs and exit",
    )
    return parser


def main() -> None:
    args = _build_arg_parser().parse_args()
    configs = can.detect_available_configs()

    if args.list:
        if not configs:
            print("[INFO] No CAN configs detected by python-can.")
        else:
            print("[INFO] Detected CAN configs:")
            for i, cfg in enumerate(configs, start=1):
                print(f"  {i}. {cfg}")
        raise SystemExit(0)

    interface = args.interface
    channel = args.channel
    if not interface or not channel:
        print("[INFO] Please specify --interface and --channel explicitly.")
        if not configs:
            print("[INFO] No CAN configs detected by python-can.")
        else:
            print("[INFO] Detected CAN configs:")
            for i, cfg in enumerate(configs, start=1):
                print(f"  {i}. {cfg}")
        raise SystemExit(
            "Missing required args. Example: python test.py --interface slcan --channel COM21"
        )

    if (args.expected_src >= 0) and (args.expected_src > 14):
        raise SystemExit("--expected-src must be in range 0..14, or -1 for any source.")

    ok = test_receive_broadcast(
        interface=interface,
        channel=str(channel),
        receive_seconds=args.seconds,
        recv_timeout=args.recv_timeout,
        nominal_bitrate=args.bitrate,
        expected_src=None if args.expected_src < 0 else args.expected_src,
    )
    raise SystemExit(0 if ok else 1)


if __name__ == "__main__":
    main()
