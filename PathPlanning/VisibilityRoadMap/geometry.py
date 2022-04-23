class Geometry:

    class Point:
        def __init__(self, x, y):
            self.x = x
            self.y = y

    @staticmethod
    def is_seg_intersect(p1, q1, p2, q2):

        def on_segment(p, q, r):
            if ((q.x <= max(p.x, r.x)) and (q.x >= min(p.x, r.x)) and
                    (q.y <= max(p.y, r.y)) and (q.y >= min(p.y, r.y))):
                return True
            return False

        def orientation(p, q, r):
            val = (float(q.y - p.y) * (r.x - q.x)) - (
                    float(q.x - p.x) * (r.y - q.y))
            if val > 0:
                return 1
            if val < 0:
                return 2
            return 0

        # Find the 4 orientations required for
        # the general and special cases
        o1 = orientation(p1, q1, p2)
        o2 = orientation(p1, q1, q2)
        o3 = orientation(p2, q2, p1)
        o4 = orientation(p2, q2, q1)

        if (o1 != o2) and (o3 != o4):
            return True
        if (o1 == 0) and on_segment(p1, p2, q1):
            return True
        if (o2 == 0) and on_segment(p1, q2, q1):
            return True
        if (o3 == 0) and on_segment(p2, p1, q2):
            return True
        if (o4 == 0) and on_segment(p2, q1, q2):
            return True

        return False
