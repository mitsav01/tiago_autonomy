#!/usr/bin/env python3
import sys
import time
import csv
import rclpy
from rclpy.node import Node
from rosidl_runtime_py.utilities import get_message


class UniversalJitterBenchmarker(Node):
    def __init__(self, topic_name, output_csv, max_samples=30000):
        super().__init__('jitter_benchmarker')
        self.topic_name = topic_name
        self.output_csv = output_csv
        self.max_samples = max_samples
        self.timestamps = []
        self.prev_time = None

        self.get_logger().info(f"Probing topic: '{self.topic_name}'...")
        
        # 1. Resolve message type dynamically from the graph
        msg_type_str = self.resolve_topic_type(self.topic_name)
        if not msg_type_str:
            self.get_logger().error(
                f"Topic '{self.topic_name}' is not currently being published. "
                "Verify your robot/sim is running with: ros2 topic list"
            )
            raise SystemExit(1)

        msg_cls = get_message(msg_type_str)
        self.get_logger().info(f"Resolved type: {msg_type_str}")
        self.get_logger().info(f"Collecting {self.max_samples} samples -> Output: {self.output_csv}")

        # 2. Subscribe to incoming messages
        self.sub = self.create_subscription(
            msg_cls, self.topic_name, self.cb, 10
        )

    def resolve_topic_type(self, topic):
        # Wait up to 5 seconds for topic discovery in DDS graph
        start = time.time()
        while time.time() - start < 5.0:
            for name, types in self.get_topic_names_and_types():
                if name == topic and types:
                    return types[0]
            time.sleep(0.2)
        return None

    def cb(self, _):
        now = time.clock_gettime_ns(time.CLOCK_MONOTONIC)
        if self.prev_time is not None:
            dt_us = (now - self.prev_time) / 1000.0  # microseconds
            self.timestamps.append(dt_us)

            count = len(self.timestamps)
            if count % 1000 == 0:
                self.get_logger().info(f"Progress: {count}/{self.max_samples} samples (current dt: {dt_us:.1f} µs)")

            if count >= self.max_samples:
                self.save_and_exit()
        self.prev_time = now

    def save_and_exit(self):
        self.get_logger().info(f"Writing {len(self.timestamps)} samples to '{self.output_csv}'...")
        with open(self.output_csv, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['dt_us'])
            for dt in self.timestamps:
                writer.writerow([dt])
        self.get_logger().info("Benchmark complete!")
        raise SystemExit(0)


def main():
    if len(sys.argv) < 3:
        print("Usage: python3 jitter_benchmark.py <topic_name> <output_csv> [max_samples]")
        print("Example: python3 jitter_benchmark.py /joint_states no_rtt_stressed.csv 30000")
        sys.exit(1)

    topic = sys.argv[1]
    csv_file = sys.argv[2]
    samples = int(sys.argv[3]) if len(sys.argv) > 3 else 30000

    rclpy.init()
    try:
        node = UniversalJitterBenchmarker(topic, csv_file, max_samples=samples)
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()