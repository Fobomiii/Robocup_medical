"""QoS profiles shared by latency-sensitive navigation sensors."""

from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)


def low_latency_sensor_qos() -> QoSProfile:
    """Keep only the newest best-effort sample instead of processing backlog."""

    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )
