#from brep.geometry import Point
#
#class SnappingGrid:
#    def __init__(self):
#        self.grid = {}
#        self.first_pt = None
#        self.set_precision(3)
#        self.set_grid_size(1)
#    
#    def set_precision(self, precision):
#        if self.first_pt is None:
#            self.precision = pow(10, precision)
#        else:
#            raise ValueError('You can not change precision after points were inserted')
#    
#    def set_grid_size(self, threshold):
#        if self.first_pt is None:
#            self.threshold = threshold
#            self.threshold_pow = pow(self.threshold, 2)
#        else:
#            raise ValueError('You can not change precision after points were inserted')
#    
#    def __str__(self):
#        return "{0}".format(self.grid)
#
#    def make_key(self, point):
#        pt_precise = int(round(point[0] * self.precision)), int(round(point[1] * self.precision))
#        if self.first_pt is None:
#            self.first_pt = pt_precise
#        pt_precise_grid = (( ((pt_precise[0] - self.first_pt[0]) // self.threshold) * self.threshold, 
#                             ((pt_precise[1] - self.first_pt[1]) // self.threshold) * self.threshold) )
#        key = self._find_similar_points(pt_precise_grid, pt_precise)
#        if key is not None:
#            return key
#        else:
#            self.grid[pt_precise_grid] = []
#            self.grid[pt_precise_grid].append(pt_precise)
#            return pt_precise_grid
#
#    def _find_similar_points(self, kernel, pt_new):
##        print "k:", kernel[0]
##        print "t:", self.threshold
#        try:
#            for i in range(kernel[0] - self.threshold, kernel[0] + 2 * self.threshold, self.threshold):
#                for j in range(kernel[1] - self.threshold, kernel[1] + 2 * self.threshold, self.threshold):
#                    if (i, j) in self.grid:
#                        for pt in self.grid[(i,j)]:
#                            dx = pt_new[0] - pt[0]
#                            dy = pt_new[1] - pt[1]
#                            dist = pow(dx, 2) + pow(dy,2)
#                            if dist <= self.threshold_pow:
#                                if dist != 0:
#                                    self.grid[(i, j)].append(pt_new)
#                                return (i, j)
#        except:
#            print "k:", kernel
#            print "t:", self.threshold
#            print self.first_pt
#            raise
#
#if __name__ == "__main__":
#    snapper = SnappingGrid()
#    snapper.set_precision(3)
#    snapper.set_grid_size(25)
#    pts = [Point(10.12345, 1.12345), Point(10.1345, 1.12345),Point(10.1345, 1.136)
#    , Point(10.145, 1.12345), Point(10.15, 1.12345)
#    , Point(10.5, 1.12345), Point(10.1, 1.12345)
#    , Point(10.1, 1.1), Point(10.5001, 1.12345), Point(10.49999, 1.12345) 
#    ]
#    
#    for i in range(0,50):
#        pts.append(Point(10.1345, i*0.001+1.13))
#    for pt in pts:
#        print "...", pt, snapper.make_key(pt)
#    print snapper
