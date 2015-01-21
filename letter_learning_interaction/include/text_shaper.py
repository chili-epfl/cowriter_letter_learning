import logging; logger = logging.getLogger("text_shaper")
logger.setLevel(logging.DEBUG)

import numpy
from scipy import interpolate

from shape_learning.shape_modeler import ShapeModeler #for normaliseShapeHeight()

SIZESCALE_HEIGHT = 0.035   #Desired height of shape (metres) (because of grid used by shape_display_manager)
SIZESCALE_WIDTH = 0.023    #Desired width of shape (metres) (@todo this should be set by shape_display_manager)

LETTER_SCALES = {'a': 0.4,
                 'c': 0.4,
                 'e': 0.4,
                 'i': 0.3,
                 'm': 0.4,
                 'n': 0.4,
                 'o': 0.4,
                 'r': 0.4,
                 's': 0.4,
                 'u': 0.4,
                 'v': 0.4,
                 'x': 0.4}


class ShapedWord:
    """ Container for the paths of the letters of a given word.
    It also exposes the bounding boxes of each letters and of the whole
    word.
    """
    def __init__(self, word, paths, origin = None):
        self.word = word
        self._paths = paths

        self.bounding_boxes = self._compute_bbs()
        self.global_bounding_box = self._compute_global_bb()

        self.origin = origin if origin is not None else [0,0]

    def get_letters_paths(self, absolute=True):

        if absolute:
            return [[(x + self.origin[0], y + self.origin[1]) for x,y in path] for path in self._paths]
        else:
            return self._paths

    def get_letters_bounding_boxes(self, absolute=True):

        if absolute:
            return [(x1 + self.origin[0], y1 + self.origin[1], x2 + self.origin[0], y2 + self.origin[1]) for x1,y1,x2,y2 in self.bounding_boxes]
        else:
            return self.bounding_boxes

    def get_global_bb(self, absolute=True):

        if absolute:
            x1, y1, x2, y2 = self.global_bounding_box
            return x1 + self.origin[0], y1 + self.origin[1], x2 + self.origin[0], y2 + self.origin[1]
        else:
            return self.global_bounding_box

    @staticmethod
    def compute_boundingbox(path):

        x_min = 2000
        y_min = 2000
        x_max = 0
        y_max = 0

        for x,y in path:

            if x < x_min:
                x_min = x
            if y < y_min:
                y_min = y

            if x > x_max:
                x_max = x
            if y > y_max:
                y_max = y

        return x_min, y_min, x_max, y_max

    def _compute_bbs(self):

        bbs = []

        for path in self._paths:
            bbs.append(ShapedWord.compute_boundingbox(path))

        return bbs

    def _compute_global_bb(self):

        gx_min = 2000
        gy_min = 2000
        gx_max = 0
        gy_max = 0

        for x_min,y_min,x_max,y_max in self.bounding_boxes:

            if x_min < gx_min:
                gx_min = x_min
            if y_min < gy_min:
                gy_min = y_min

            if x_max > gx_max:
                gx_max = x_max
            if y_max > gy_max:
                gy_max = y_max

        return gx_min, gy_min, gx_max, gy_max

    def downsample(self, downsampling_factor):

        downsampled_paths = []

        for path in self._paths:
            x_shape = [p[0] for p in path]
            y_shape = [p[1] for p in path]
            #make shape have the appropriate number of points
            t_current = numpy.linspace(0, 1, len(x_shape))
            t_desired = numpy.linspace(0, 1, len(x_shape) / downsampling_factor)
            f = interpolate.interp1d(t_current, x_shape[:], kind='cubic')
            x_shape = f(t_desired)
            f = interpolate.interp1d(t_current, y_shape[:], kind='cubic')
            y_shape = f(t_desired)

            path = zip(x_shape, y_shape)

            downsampled_paths.append(path)

        self._paths = downsampled_paths

        self.bounding_boxes = self._compute_bbs()
        self.global_bounding_box = self._compute_global_bb()

    def _isinbb(self, x,y, bb):
            x1, y1, x2, y2 = bb
            return x1 <= x <= x2 and y1 <= y <= y2

    def ispointonword(self, x, y):
        """
        :returns: (False, None, None) if the point is not within the bounding
        box of one of the letter, (True, <letter>, <bounding box>) if the point
        lays on the bounding box of one of the letters.

        """

        for i, bb in enumerate(self.bounding_boxes):
            if self._isinbb(x - self.origin[0], y - self.origin[1], bb):
                return True, self.word[i], self.get_letters_bounding_boxes()[i]

        return False, None, None


class TextShaper:

    def shapeWord(self, word, downsampling_factor=None):
        """Assembles the paths of the letters of the given word into a global shape.

        :param word: a ShapeLearnerManager instance for the current word
        :param downsampling_factor: if provided, the final shape of each letter
        is (independantly) resampled to (nb_pts / downsampling_factor) points

        :returns: a ShapedWord that contains the path of individual letters
        """
        
        paths = []

        for shape in word.shapesOfCurrentCollection():

            path = []

            scale_factor = LETTER_SCALES.get(shape.shapeType, 1.0)

            glyph = ShapeModeler.normaliseShapeHeight(shape.path)
            numPointsInShape = len(glyph)/2  

            x_shape = glyph[0:numPointsInShape].flatten().tolist()
            y_shape = glyph[numPointsInShape:].flatten().tolist()

            # connect the letter to the ending point of the previous one
            offset_x = offset_y = 0
            if len(paths) > 0:
                offset_x = paths[-1][-1][0] - x_shape[0] * SIZESCALE_WIDTH * scale_factor
                offset_y = paths[-1][-1][1] + y_shape[0] * SIZESCALE_HEIGHT * scale_factor

            for i in range(numPointsInShape):
                x = x_shape[i] * SIZESCALE_WIDTH * scale_factor
                y = -y_shape[i] * SIZESCALE_HEIGHT * scale_factor

                x += offset_x
                y += offset_y

                path.append((x,y))

            paths.append(path)


        return ShapedWord(word.currentCollection, paths)

class ScreenManager:

    def __init__(self, width, height):
        """
        :param width: width, in meters, of the writing zone
        :param height: height, in meters, of the writing zone
        """
        self.width = width
        self.height = height

        self.words = []

    def clear(self):
        self.words = []

    def place_word(self, shaped_word):
        """ Note that this method *modifies* its parameter!
        """
        shaped_word.origin = [self.width * 0.3, self.height * 0.6]
        self.words.append(shaped_word)
        return shaped_word

    def closest_letter(self, x, y):
        """ Returns the letter (+ boudning box) on the screen the closest to
        (x,y) in screen coordinates, or None if no word has been drawn.

        """
        if not self.words:
            logger.debug("Closest letter: no word drawn yet!")
            return None, None

        for word in self.words:
            on_letter, letter, bb = word.ispointonword(x, y)
            if on_letter:
                logger.debug("Closest letter: on top of '%s' bounding box", letter)
                return letter, bb

        # not on top of a bounding-box: compute distances to each letter, and
        # return the closest one

        distances = {}

        for word in self.words:
            for i, bb in enumerate(word.get_letters_bounding_boxes()):
                x1,y1,x2,y2 = bb
                bbx =  float(x2 + x1)/2
                bby =  float(y2 + y1)/2
                distance = (x - bbx) * (x - bbx) + (y - bby) * (y - bby)
                distances[distance] = (word.word[i], bb) # store the letter with its distance

        shortest_distance = sorted(distances.keys())[0]
        letter, bb = distances[shortest_distance]

        logger.debug("Closest letter: '%s'", letter)
        return letter, bb

    def find_letter(self, path):

        x,y = ShapeModeler.getShapeCentre(path)
        return self.closest_letter(x, y)
