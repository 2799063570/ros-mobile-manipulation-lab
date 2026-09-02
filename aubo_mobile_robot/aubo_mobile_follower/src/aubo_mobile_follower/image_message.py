"""ROS image-message helpers independent of OpenCV's numeric type constants."""

import numpy as np
from sensor_msgs.msg import Image


def bgr8_to_imgmsg(image, header=None):
    """Return a ``bgr8`` Image without calling ``cv_bridge.cv2_to_imgmsg``.

    Noetic's cv_bridge may be compiled against an older OpenCV than the Python
    ``cv2`` module loaded at runtime.  OpenCV 5 changed numeric matrix type
    constants, which makes cv_bridge raise ``KeyError`` for an otherwise valid
    uint8 BGR image.  The ROS message layout is unambiguous, so construct it
    directly instead of depending on those constants.
    """
    image = np.asarray(image)
    if image.dtype != np.uint8:
        raise ValueError("bgr8 image must have dtype uint8, got {}".format(image.dtype))
    if image.ndim != 3 or image.shape[2] != 3:
        raise ValueError(
            "bgr8 image must have shape (height, width, 3), got {}".format(
                image.shape
            )
        )

    image = np.ascontiguousarray(image)
    message = Image()
    if header is not None:
        message.header = header
    message.height = image.shape[0]
    message.width = image.shape[1]
    message.encoding = "bgr8"
    message.is_bigendian = 0
    message.step = image.shape[1] * image.shape[2]
    message.data = image.tobytes()
    return message
