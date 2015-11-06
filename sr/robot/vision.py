import pykoki, threading, time, functools, re, subprocess
from collections import namedtuple
from pykoki import CameraParams, Point2Df, Point2Di

C500_focal_length = {
    (1280, 1024): (1088.6744696128017, 1088.6744696128017),
    (1280, 800): (1077.3190248161634, 1077.3190248161634),
    (1280, 720): (1087.9702553611371, 1087.9702553611371),
    (960, 720): (810.09860332369828, 810.09860332369828),
    (800, 600): (672.49473805241894, 672.49473805241894),
    (640, 480): (539.30891554396362, 539.30891554396362),
    (640, 400): (536.63303379131355, 536.63303379131355),
    (640, 360): (538.60066822927899, 538.60066822927899),
    (352, 288): (302.3574089608714, 302.3574089608714),
    (320, 240): (267.87289797043974, 267.87289797043974),
    (176, 144): (145.6217107354399, 145.6217107354399),
    (160, 120): (129.71444255178932, 129.71444255178932)
}

C270_focal_length = {
    (1280, 960): (1402.4129693403379, 1402.4129693403379),
    (1280, 720): (1403.6700720661784, 1403.6700720661784),
    (1024, 576): (1124.3241302755343, 1124.3241302755343),
    (960, 720):  (1051.3506745692532, 1051.3506745692532),
    (960, 544):  (1050.6160916576887, 1050.6160916576887),
    (864, 480):  (940.64475443969184, 940.64475443969184),
    (800, 600):  (870.49134233524376, 870.49134233524376),
    (800, 448):  (870.24502453110301, 870.24502453110301),
    (752, 416):  (813.03563104383659, 813.03563104383659),
    (640, 480):  (793.97214065079106, 793.97214065079106),
    (544, 288):  (593.33938236047481, 593.33938236047481),
    (432, 240):  (473.402174937062,   473.402174937062),
    (352, 288):  (416.51679809127182, 416.51679809127182),
    (320, 240):  (396.10516141661941, 396.10516141661941),
    (320, 176):  (339.97100058087398, 339.97100058087398),
    (176, 144):  (204.85920172232522, 204.85920172232522),
    (160, 120):  (195.20341230615605, 195.20341230615605)
}

focal_length_lut = {
    (0x046d, 0x0807): C500_focal_length,
    (0x046d, 0x0825): C270_focal_length
}

MARKER_ARENA, MARKER_ROBOT, MARKER_TOP, MARKER_BOTTOM, MARKER_SIDE = 'arena', 'robot', 'top', 'bottom', 'side'
TOKEN_NETS = ("NET_A", "NET_B", "NET_C")

marker_offsets = {
    MARKER_ARENA: 0,
    MARKER_ROBOT: 28,
}
MARKER_TOKEN_OFFSET = 32

marker_sizes = {
    MARKER_ARENA: 0.25 * (10.0/12),
    MARKER_ROBOT: 0.1 * (10.0/12),
    MARKER_TOP: 0.2 * (10.0/12),
    MARKER_BOTTOM: 0.2 * (10.0/12),
    MARKER_SIDE: 0.2 * (10.0/12)
}

MarkerInfo = namedtuple( "MarkerInfo", "code marker_type offset size token_net" )
ImageCoord = namedtuple( "ImageCoord", "x y" )
WorldCoord = namedtuple( "WorldCoord", "x y z" )
PolarCoord = namedtuple( "PolarCoord", "length rot_x rot_y" )
Orientation = namedtuple( "Orientation", "rot_x rot_y rot_z" )
Point = namedtuple( "Point", "image world polar" )

TOKEN_MARKERS = [(MARKER_TOP, 1),(MARKER_BOTTOM, 1),(MARKER_SIDE,4)]

# Number of markers per group
marker_group_counts = {
    "dev": [ ( MARKER_ARENA, 28 ),
             ( MARKER_ROBOT, 4 ) ],
    "comp": [ ( MARKER_ARENA, 28 ),
              ( MARKER_ROBOT, 4 ) ],
}

def create_marker_lut(offset, counts):
    lut = {}
    for genre, num in counts:
        for n in range(0,num):
            base_code = marker_offsets[genre] + n
            real_code = offset + base_code
            m = MarkerInfo( code = base_code,
                            marker_type = genre,
                            offset = n,
                            size = marker_sizes[genre],
                            token_net = None )
            lut[real_code] = m

    # Now add on the token markers
    base_code = MARKER_TOKEN_OFFSET
    for net in TOKEN_NETS:
        for face in (MARKER_TOP, MARKER_BOTTOM) + (MARKER_SIDE,)*4:
            real_code = offset + base_code

            m = MarkerInfo( code = base_code,
                            marker_type = face,
                            offset = None,
                            size = marker_sizes[face],
                            token_net = net )
            lut[real_code] = m

            base_code += 1

    return lut

marker_luts = { "dev": { "A": create_marker_lut(0, marker_group_counts["dev"]),
                         "B": create_marker_lut(0, marker_group_counts["dev"]) },
                "comp": { "A": create_marker_lut(100, marker_group_counts["comp"]),
                          "B": create_marker_lut(150, marker_group_counts["comp"]) } }

MarkerBase = namedtuple( "Marker", "info timestamp res vertices centre orientation" ) 
class Marker(MarkerBase):
    def __init__( self, *a, **kwd ):
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
    def __init__(self, camdev, lib):
        self.koki = pykoki.PyKoki(lib)
        self._camdev = camdev
        self.camera = self.koki.open_camera(self._camdev)

        self.camera_focal_length = None
        self._init_focal_length()

        # Lock for the use of the vision
        self.lock = threading.Lock()
        self.lock.acquire()

        self._res = None
        self._buffers = None
        self._streaming = False

        # Default to 800x600        
        self._set_res( (800,600) )
        self._start()
        self.lock.release()

    def __del__(self):
        self._stop()

    def _init_focal_length(self):
        vendor_product_re = re.compile(".* ([0-9A-Za-z]+):([0-9A-Za-z]+) ")
        p = subprocess.Popen(["lsusb"], stdout=subprocess.PIPE)
        stdout, _ = p.communicate()
        for line in stdout.splitlines():
            match = vendor_product_re.match(line)
            id = tuple(int(x, 16) for x in match.groups())
            if id in focal_length_lut:
               self.camera_focal_length = focal_length_lut[id]
               return

    def _set_res(self, res):
        "Set the resolution of the camera if different to what we were"
        if res == self._res:
            "Resolution already the requested one"
            return

        was_streaming = self._streaming
        if was_streaming:
            self._stop()

        # The camera goes into a strop if we don't close and open again
        del self.camera
        self.camera = self.koki.open_camera(self._camdev)

        self.camera.format = self.koki.v4l_create_YUYV_format( res[0], res[1] )

        fmt = self.camera.format
        width = fmt.fmt.pix.width
        height = fmt.fmt.pix.height

        if width != res[0] or height != res[1]:
            raise ValueError( "Unsupported image resolution" )
        self._res = (width, height)

        if was_streaming:
            self._start()

    def _stop(self):
        self.camera.stop_stream()
        self._streaming = False

    def _start(self):
        self.camera.prepare_buffers(1)
        self.camera.start_stream()
        self._streaming = True

    def _width_from_code(self, lut, code):
        if code not in lut:
            # We really want to ignore these...
            return 0.1

        return lut[code].size

    def see(self, mode, arena, res, stats):
        self.lock.acquire()
        self._set_res(res)

        acq_time = time.time()

        timer = Timer()
        times = {}

        with timer:
            frame = self.camera.get_frame()
        times["cam"] = timer.time

        with timer:
            img = self.koki.v4l_YUYV_frame_to_grayscale_image( frame, self._res[0], self._res[1] )
        times["yuyv"] = timer.time

        # Now that we're dealing with a copy of the image, release the camera lock
        self.lock.release()

        params = CameraParams( Point2Df( self._res[0]/2,
                                         self._res[1]/2 ),
                               Point2Df( *self.camera_focal_length[ self._res ] ),
                               Point2Di( *self._res ) )

        with timer:
            markers = self.koki.find_markers_fp( img,
                                                 functools.partial( self._width_from_code, marker_luts[mode][arena] ),
                                                 params )
        times["find_markers"] = timer.time

        srmarkers = []
        for m in markers:
            if m.code not in marker_luts[mode][arena]:
                "Ignore other sets of codes"
                continue

            info = marker_luts[mode][arena][int(m.code)]

            vertices = []
            for v in m.vertices:
                vertices.append( Point( image = ImageCoord( x = v.image.x,
                                                            y = v.image.y ),
                                        world = WorldCoord( x = v.world.x,
                                                            y = v.world.y,
                                                            z = v.world.z ),
                                        # libkoki does not yet provide these coords
                                        polar = PolarCoord( 0,0,0 ) ) )

            centre = Point( image = ImageCoord( x = m.centre.image.x,
                                                y = m.centre.image.y ),
                            world = WorldCoord( x = m.centre.world.x,
                                                y = m.centre.world.y,
                                                z = m.centre.world.z ),
                            polar = PolarCoord( length = m.distance,
                                                rot_x = m.bearing.x,
                                                rot_y = m.bearing.y ) )

            orientation = Orientation( rot_x = m.rotation.x,
                                       rot_y = m.rotation.y,
                                       rot_z = m.rotation.z )

            marker = Marker( info = info,
                             timestamp = acq_time,
                             res = res,
                             vertices = vertices,
                             centre = centre,
                             orientation = orientation )
            srmarkers.append(marker)

        self.koki.image_free(img)

        if stats:
            return (srmarkers, times)

        return srmarkers
