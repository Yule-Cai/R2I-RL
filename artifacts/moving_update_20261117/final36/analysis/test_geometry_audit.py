import math
import unittest
from geometry_audit import convex_distance,convex_intersects,rectangle,transform


class GeometryAuditTests(unittest.TestCase):
    def setUp(self):
        self.square=rectangle([(0.,0.),(1.,0.),(1.,1.),(0.,1.)])

    def test_crossing_and_tangent_segments(self):
        for segment in ([(-1.,.5),(2.,.5)],[(-1.,1.),(2.,1.)],[(.5,.5),(.5,.5)]):
            self.assertTrue(convex_intersects(segment,self.square))
            self.assertEqual(convex_distance(segment,self.square),0.)

    def test_segment_beyond_corner(self):
        self.assertFalse(convex_intersects([(2.,2.),(3.,3.)],self.square))
        self.assertAlmostEqual(convex_distance([(2.,2.),(3.,3.)],self.square),math.sqrt(2))

    def test_box_distance_and_containment(self):
        self.assertAlmostEqual(convex_distance(transform(self.square,4.,5.,0.),self.square),5.)
        contained=[(.1,.1),(.9,.1),(.9,.9),(.1,.9)]
        self.assertTrue(convex_intersects(contained,self.square))

    def test_rotated_footprint(self):
        shape=[(-.2,-.1),(.2,-.1),(.2,.1),(-.2,.1)]
        rotated=transform(shape,1.5,.5,math.pi/2)
        self.assertAlmostEqual(convex_distance(rotated,self.square),.4)
        self.assertFalse(convex_intersects(rotated,self.square))

    def test_invalid_ring_order_is_rejected(self):
        with self.assertRaises(AssertionError):
            rectangle([(0.,0.),(1.,1.),(1.,0.),(0.,1.)])

    def test_ros_points_are_projected_to_planar_coordinates(self):
        self.assertEqual(rectangle([[x,y,0.] for x,y in self.square]),self.square)


if __name__=='__main__':unittest.main()
