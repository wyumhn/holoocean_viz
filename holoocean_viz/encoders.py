"""
Extra sensor encoders for holoocean_main.

holoocean_main discovers these through the 'holoocean_main.encoders' entry
point (see setup.py) and merges ENCODERS / MULTI_PUBLISHER_SENSORS into its
own tables, so sensors in a scenario JSON with "ros_publish": true get
published like the built-in ones (topic: <agent_name>/<sensor_name><suffix>).
"""
import numpy as np
from holoocean_main.interface.sensor_data_encode import SensorPublisher
from sensor_msgs.msg import Image, PointCloud2, PointField


class SidescanSonarEncoder(SensorPublisher):
    """Publishes a single ping as a 1 x RangeBins float image (raw intensities)."""

    def __init__(self, sensor_dict):
        super().__init__(sensor_dict)

        self.message_type = Image

    def encode(self, sensor_data):
        msg = self.message_type()
        msg.header.frame_id = self.socket

        ping = np.asarray(sensor_data, dtype=np.float32).ravel()
        msg.height = 1
        msg.width = ping.shape[0]
        msg.encoding = '32FC1'
        msg.is_bigendian = 0
        msg.step = msg.width * 4
        msg.data = ping.tobytes()

        return msg


class SidescanSonarWaterfallEncoder(SensorPublisher):
    """Stacks the most recent pings into a mono8 waterfall image (newest ping on top)."""

    def __init__(self, sensor_dict):
        super().__init__(sensor_dict)

        self.message_type = Image
        self.length = int(sensor_dict.get('waterfall_length', 500))
        self.waterfall = None

    def encode(self, sensor_data):
        ping = np.asarray(sensor_data, dtype=np.float32).ravel()
        if self.waterfall is None or self.waterfall.shape[1] != ping.shape[0]:
            self.waterfall = np.zeros((self.length, ping.shape[0]), dtype=np.float32)

        self.waterfall = np.roll(self.waterfall, 1, axis=0)
        self.waterfall[0] = ping

        # Normalize by the brightest return currently in the window
        peak = float(self.waterfall.max())
        scale = 255.0 / peak if peak > 0.0 else 0.0
        image = np.clip(self.waterfall * scale, 0, 255).astype(np.uint8)

        msg = self.message_type()
        msg.header.frame_id = self.socket
        msg.height = image.shape[0]
        msg.width = image.shape[1]
        msg.encoding = 'mono8'
        msg.is_bigendian = 0
        msg.step = msg.width
        msg.data = image.tobytes()

        return msg


class RaycastLidarEncoder(SensorPublisher):
    """HoloOcean returns [num_points, 5] = x, y, z (LiDAR frame), intensity, ring.

    The docstring of holoocean.sensors.RaycastLidar says world coordinates, but
    in HoloOcean 2.3.0 the points are relative to the sensor and rotate with it.
    """

    def __init__(self, sensor_dict):
        super().__init__(sensor_dict)

        self.message_type = PointCloud2
        self.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
            PointField(name='ring', offset=16, datatype=PointField.FLOAT32, count=1),
        ]

    def encode(self, sensor_data):
        points = np.asarray(sensor_data, dtype=np.float32).reshape(-1, 5)
        if points.shape[0] == 0:
            # No capture this tick (Hz < ticks_per_sec). holoocean_interface skips
            # publishing on KeyError, the same as for sensors that did not tick.
            raise KeyError(self.name)

        msg = self.message_type()
        # Sensor-relative points; lidar_bridge publishes the TF for them as <agent>/lidar_link
        msg.header.frame_id = self.name
        msg.height = 1
        msg.width = points.shape[0]
        msg.fields = self.fields
        msg.is_bigendian = False
        msg.point_step = 20
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = True
        msg.data = points.tobytes()

        return msg


ENCODERS = {
    'SidescanSonar': SidescanSonarEncoder,
    'SidescanSonarWaterfall': SidescanSonarWaterfallEncoder,
    'RaycastLidar': RaycastLidarEncoder,
}

MULTI_PUBLISHER_SENSORS = {
    'SidescanSonar': ['', 'Waterfall'],
}

# Object loaded by holoocean_main through the 'holoocean_main.encoders' entry point
PLUGIN = {
    'encoders': ENCODERS,
    'multi_publisher_sensors': MULTI_PUBLISHER_SENSORS,
}
