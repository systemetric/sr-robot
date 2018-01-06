import threading
import time
import functools
import io
from collections import namedtuple

import cv2
import numpy as np
import picamera

import pykoki
from pykoki import CameraParams, Point2Df, Point2Di

# TODO: work out which of these actually work/are useful.
# TODO: more accurate values for these.
picamera_focal_lengths = {  # fx, fy tuples
    (1920, 1080): (2320.96618182, 2323.87230303),
    (2592, 1944): (1820.3656323, 1822.644943553),
    (1296, 972): (1531.83768, 1533.75572),
    (1296, 730): (1531.83768, 1533.75572),
    (640, 480): (1531.83768, 1533.75572),
}

# Source: <https://elinux.org/Rpi_Camera_Module#Technical_Parameters_.28v.2_board.29>
FOCAL_LENGTH_MM = 4.0
SENSOR_WIDTH_MM = 3.674
SENSOR_HEIGHT_MM = 2.760

# Estimate focal lengths for resolutions we don't have a focal length for.
# These aren't particularly accurate; the estimate for 1920x1080 caused a
# length of 50 cm to be reported as 60 cm.
# <http://answers.opencv.org/question/17076/conversion-focal-distance-from-mm-to-pixels/?answer=17180#post-id-17180>
for res in picamera_focal_lengths:
    if picamera_focal_lengths[res] is None:
        # focal length in px = (focal length in mm / sensor width in mm) * image width in px
        focal_length_px_x = (FOCAL_LENGTH_MM / SENSOR_WIDTH_MM) * res[0]
        focal_length_px_y = (FOCAL_LENGTH_MM / SENSOR_HEIGHT_MM) * res[1]
        picamera_focal_lengths[res] = (focal_length_px_x, focal_length_px_y)

MARKER_ARENA, MARKER_TOKEN, MARKER_BUCKET_SIDE, MARKER_BUCKET_END = 'arena', 'token', 'bucket-side', 'bucket-end'

marker_offsets = {
    MARKER_ARENA: 0,
    MARKER_TOKEN: 32,
    MARKER_BUCKET_SIDE: 72,
    MARKER_BUCKET_END: 76,
}

# The numbers here (e.g. `0.25`) are in metres -- the 10/12 is a scaling factor
# so that libkoki gets the size of the 10x10 black/white portion (not including
# the white border), but so that humans can measure sizes including the border.
marker_sizes = {
    MARKER_ARENA: 0.25 * (10.0/12),
    MARKER_TOKEN: 0.1 * (10.0/12),
    MARKER_BUCKET_SIDE: 0.1 * (10.0/12),
    MARKER_BUCKET_END: 0.1 * (10.0/12),
}

MarkerInfo = namedtuple("MarkerInfo", "code marker_type offset size")
ImageCoord = namedtuple("ImageCoord", "x y")
WorldCoord = namedtuple("WorldCoord", "x y z")
PolarCoord = namedtuple("PolarCoord", "length rot_x rot_y")
Orientation = namedtuple("Orientation", "rot_x rot_y rot_z")
Point = namedtuple("Point", "image world polar")

# Number of markers per group
marker_group_counts = {
    "dev": [(MARKER_ARENA, 24),
            (MARKER_TOKEN, 40),
            (MARKER_BUCKET_SIDE, 4),
            (MARKER_BUCKET_END, 4)],
    "comp": [(MARKER_ARENA, 24),
             (MARKER_TOKEN, 40),
             (MARKER_BUCKET_SIDE, 4),
             (MARKER_BUCKET_END, 4)],
}


def create_marker_lut(offset, counts):
    lut = {}
    for marker_type, num_markers in counts:
        for n in range(0, num_markers):
            base_code = marker_offsets[marker_type] + n
            real_code = offset + base_code
            m = MarkerInfo(code=base_code,
                           marker_type=marker_type,
                           offset=n,
                           size=marker_sizes[marker_type])
            lut[real_code] = m
    return lut


# 0: arena
# ...
# 23: arena
# [24-31 undefined]
# 32: token
# ...
# 71: token
# 72: bucket side
# ...
# 75: bucket side
# 76: bucket end
# ...
# 79: bucket end


marker_luts = {
    "dev": {"A": create_marker_lut(0, marker_group_counts["dev"]),
            "B": create_marker_lut(0, marker_group_counts["dev"])},
    "comp": {"A": create_marker_lut(0, marker_group_counts["comp"]),
             "B": create_marker_lut(0, marker_group_counts["comp"])}
}

MarkerBase = namedtuple("Marker", "info timestamp res vertices centre orientation")


class Marker(MarkerBase):
    def __init__(self, *a, **kwd):
        # Aliases
        self.dist = self.centre.polar.length
        self.rot_y = self.centre.polar.rot_y


class Timer(object):
    def __enter__(self):
        self.start = time.time()

    def __exit__(self, t, v, tb):
        self.time = time.time() - self.start
        return False


class Vision(object):
    def __init__(self, lib=None, res=(1920, 1080)):
        if lib is not None:
            self.koki = pykoki.PyKoki(lib)
        else:
            self.koki = pykoki.PyKoki()
        self.camera = picamera.PiCamera(resolution=res)

        # Lock for the use of the vision
        self.lock = threading.Lock()

    def __del__(self):
        self.camera.close()

    def _set_res(self, res):
        """Set the resolution of the camera if different to what we were"""
        if res == self.camera.resolution:
            # Resolution already the requested one
            return

        try:
            self.camera.resolution = res
        except Exception as e:
            raise ValueError("Setting camera resolution failed with {}".format(type(e)))

        actual = self.camera.resolution

        if res != actual:
            raise ValueError("Unsupported image resolution {0} (got: {1})".format(res, actual))

    def _width_from_code(self, lut, code):
        if code not in lut:
            # We really want to ignore these...
            return 0.1

        return lut[code].size

    def see(self, mode, arena, res=None, stats=False, save=True, fast_capture=True):
        self.lock.acquire()
        if res is not None:
            self._set_res(res)

        acq_time = time.time()

        timer = Timer()
        times = {}

        stream = io.BytesIO()

        with timer:
            self.camera.capture(stream, format="jpeg", use_video_port=fast_capture)
        times["cam"] = timer.time

        with timer:
            # Turn the stream of bytes into a NumPy array.
            jpeg_data = np.fromstring(stream.getvalue(), dtype=np.uint8)
            # Decode the image from JPEG to a 2D NumPy array.
            # `0` for the second argument indicates that the result should be greyscale.
            image = cv2.imdecode(jpeg_data, 0)
            if save:
                cv2.imwrite("/tmp/marker.jpg", image)
            # Create an IplImage header for the image.
            # (width, height), depth, num_channels
            ipl_image = cv2.cv.CreateImageHeader((image.shape[1], image.shape[0]), cv2.cv.IPL_DEPTH_8U, 1)
            # Put the actual image data in the IplImage.
            # The third argument is the row length ("step").
            # Note that pykoki will automatically free `ipl_image` after the markers are obtained from it.
            cv2.cv.SetData(ipl_image, image.tobytes(), image.dtype.itemsize * image.shape[1])
            # Make sure the image data is actually in the IplImage. Don't touch this line!
            ipl_image.tostring()
        times["manipulation"] = timer.time

        # Now that we're dealing with a copy of the image, release the camera lock
        self.lock.release()

        params = CameraParams(Point2Df(self.camera.resolution[0] / 2,
                                       self.camera.resolution[1] / 2),
                              Point2Df(*picamera_focal_lengths[self.camera.resolution]),
                              Point2Di(*self.camera.resolution))

        with timer:
            markers = self.koki.find_markers_fp(ipl_image,
                                                functools.partial(self._width_from_code, marker_luts[mode][arena]),
                                                params)
        times["find_markers"] = timer.time

        srmarkers = []
        for m in markers:
            if m.code not in marker_luts[mode][arena]:
                # Ignore other sets of codes
                continue

            info = marker_luts[mode][arena][int(m.code)]

            vertices = []
            for v in m.vertices:
                vertices.append(Point(image=ImageCoord(x=v.image.x,
                                                       y=v.image.y),
                                      world=WorldCoord(x=v.world.x,
                                                       y=v.world.y,
                                                       z=v.world.z),
                                      # libkoki does not yet provide these coords
                                      polar=PolarCoord(0, 0, 0)))

            num_quarter_turns = int(m.rotation_offset / 90)
            num_quarter_turns %= 4

            vertices = vertices[num_quarter_turns:] + vertices[:num_quarter_turns]

            centre = Point(image=ImageCoord(x=m.centre.image.x,
                                            y=m.centre.image.y),
                           world=WorldCoord(x=m.centre.world.x,
                                            y=m.centre.world.y,
                                            z=m.centre.world.z),
                           polar=PolarCoord(length=m.distance,
                                            rot_x=m.bearing.x,
                                            rot_y=m.bearing.y))

            orientation = Orientation(rot_x=m.rotation.x,
                                      rot_y=m.rotation.y,
                                      rot_z=m.rotation.z)

            marker = Marker(info=info,
                            timestamp=acq_time,
                            res=res,
                            vertices=vertices,
                            centre=centre,
                            orientation=orientation)
            srmarkers.append(marker)

        if stats:
            return (srmarkers, times)

        return srmarkers
