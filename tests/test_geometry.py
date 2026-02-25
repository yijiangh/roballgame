import math
import unittest

from src.mockup import CircleObstacle, RectObstacle, SegmentObstacle, DOT_R


class GeometryTests(unittest.TestCase):
    def test_circle_distance(self):
        c = CircleObstacle(0.0, 0.0, 10.0)
        d, n = c.dist_and_normal((20.0, 0.0))
        self.assertAlmostEqual(d, 20.0 - (10.0 + DOT_R), places=6)
        self.assertAlmostEqual(n[0], 1.0, places=6)
        self.assertAlmostEqual(n[1], 0.0, places=6)

    def test_rect_outside(self):
        r = RectObstacle(0.0, 0.0, 10.0, 10.0)
        d, n = r.dist_and_normal((40.0, 0.0))
        self.assertGreater(d, 0.0)
        self.assertAlmostEqual(n[0], 1.0, places=6)
        self.assertAlmostEqual(n[1], 0.0, places=6)

    def test_rect_inside(self):
        r = RectObstacle(0.0, 0.0, 10.0, 10.0)
        d, n = r.dist_and_normal((5.0, 5.0))
        self.assertLess(d, 0.0)
        self.assertAlmostEqual(math.hypot(n[0], n[1]), 1.0, places=6)

    def test_segment_distance(self):
        s = SegmentObstacle(0.0, 0.0, 10.0, 0.0)
        d, n = s.dist_and_normal((5.0, 10.0))
        self.assertAlmostEqual(d, 10.0 - DOT_R, places=6)
        self.assertAlmostEqual(n[0], 0.0, places=6)
        self.assertAlmostEqual(n[1], 1.0, places=6)


if __name__ == "__main__":
    unittest.main()
