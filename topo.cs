using System;
using System.Collections.Generic;

using CamBam.CAD;
using CamBam.Geom;

using Tree4;

namespace Matmill
{
    class Topographer
    {
        private const double T4_MARGIN = 1.0;

        private readonly Polyline _outline;
        private readonly Polyline[] _islands;
        private readonly T4 _t4;
        bool is_t4_populated = false;


        public Point2F Min
        {
            get { return new Point2F(_t4.Rect.Xmin + T4_MARGIN, _t4.Rect.Ymin + T4_MARGIN); }
        }

        public Point2F Max
        {
            get { return new Point2F(_t4.Rect.Xmax - T4_MARGIN, _t4.Rect.Ymax - T4_MARGIN); }
        }

        private void populate_t4()
        {
            if (is_t4_populated)
                return;
            insert_in_t4(_outline);
            foreach (Polyline island in _islands)
                insert_in_t4(island);
            is_t4_populated = true;
        }

        private void insert_in_t4(Polyline p)
        {
            for (int i = 0; i < p.NumSegments; i++)
            {
                object seg = p.GetSegment(i);
                T4_rect rect;

                if (seg is Line2F)
                {
                    Line2F line = ((Line2F)seg);
                    rect = new T4_rect(Math.Min(line.p1.X, line.p2.X),
                                        Math.Min(line.p1.Y, line.p2.Y),
                                        Math.Max(line.p1.X, line.p2.X),
                                        Math.Max(line.p1.Y, line.p2.Y));
                }
                else if (seg is Arc2F)
                {
                    Point2F min = Point2F.Undefined;
                    Point2F max = Point2F.Undefined;
                    ((Arc2F)seg).GetExtrema(ref min, ref max);
                    rect = new T4_rect(min.X, min.Y, max.X, max.Y);
                }
                else
                {
                    throw new Exception("unknown segment type");
                }

                _t4.Add(rect, seg);
            }
        }

        public bool Is_line_inside_region(Line2F line, bool should_analize_inner_intersections, double tolerance)
        {
            if (!_outline.PointInPolyline(line.p1, tolerance))
                return false;     // p1 is outside of outer curve boundary
            if (!_outline.PointInPolyline(line.p2, tolerance))
                return false;  // p2 is outside of outer curve boundary
            if (should_analize_inner_intersections && _outline.LineIntersections(line, tolerance).Length != 0)
                return false; // both endpoints are inside, but there are intersections, outer curve must be concave

            foreach (Polyline island in _islands)
            {
                if (island.PointInPolyline(line.p1, tolerance))
                    return false;  // p1 is inside hole
                if (island.PointInPolyline(line.p2, tolerance))
                    return false;  // p2 is inside hole
                if (should_analize_inner_intersections && island.LineIntersections(line, tolerance).Length != 0)
                    return false; // p1, p2 are outside hole, but there are intersections
            }
            return true;
        }

        public List<Point2F> sample_curve(Polyline p, double step)
        {
            // divide curve evenly. There is a bug in CamBam's divide by step routine (duplicate points), while 'divide to n equal segments' should work ok.
            // execution speed may be worse, but who cares
            double length = p.GetPerimeter();
            int nsegs = (int)Math.Max(Math.Ceiling(length / step), 1);

            List<Point2F> points = new List<Point2F>();
            foreach (Point3F pt in PointListUtils.CreatePointlistFromPolyline(p, nsegs).Points)
                points.Add((Point2F)pt);

            return points;
        }

        public List<Point2F> sample_curve_exact(Polyline p, double step)
        {
            List<Point2F> points = new List<Point2F>();
            foreach (Point3F pt in PointListUtils.CreatePointlistFromPolylineStep(p, step).Points)
                points.Add((Point2F)pt);

            Point2F last_sample = points[points.Count - 1];
            Point2F poly_end = (Point2F)p.LastPoint;

            if (last_sample.DistanceTo(poly_end) > step * 0.001)
                points.Add(poly_end);

            return points;
        }

        public double Get_dist_to_wall(Point2F pt)
        {
            populate_t4();

            double radius = double.MaxValue;
            foreach (object item in _t4.Get_nearest_objects(pt.X, pt.Y))
            {
                double dist = 0;
                if (item is Line2F)
                    ((Line2F)item).NearestPoint(pt, ref dist);
                else
                    ((Arc2F)item).NearestPoint(pt, ref dist);
                if (dist < radius)
                    radius = dist;
            }

            return radius;
        }

        public bool Is_line_inside_region(Line2F line, double tolerance)
        {
            return Is_line_inside_region(line, true, tolerance);
        }

        public List<Point2F> Get_samples(double sample_step)
        {
            List<Point2F> plist = new List<Point2F>();

            plist.AddRange(sample_curve(_outline, sample_step));
                foreach (Polyline p in _islands)
                    plist.AddRange(sample_curve(p, sample_step));

            return plist;
        }

        public List<Point2F> Get_samples_exact(double sample_step)
        {
            List<Point2F> plist = new List<Point2F>();

            plist.AddRange(sample_curve_exact(_outline, sample_step));
            foreach (Polyline p in _islands)
                plist.AddRange(sample_curve_exact(p, sample_step));

            return plist;
        }

        public bool Build_medial_tree(Medial_branch tree, double sample_step, double general_tolerance, Point2F startpoint, double min_dist_to_wall, bool startpoint_is_a_hint)
        {
            List<Point2F> samples = Get_samples(sample_step);
            Logger.log("got {0} points", samples.Count);

            return Medial_builder.Build(tree, this, samples, general_tolerance, startpoint, min_dist_to_wall, startpoint_is_a_hint);
        }

        public Topographer(Polyline outline, Polyline[] islands)
        {
            _outline = outline;
            _islands = islands;

            Point3F min = Point3F.Undefined;
            Point3F max = Point3F.Undefined;

            _outline.GetExtrema(ref min, ref max);

            _t4 = new T4(new T4_rect(min.X - T4_MARGIN, min.Y - T4_MARGIN, max.X + T4_MARGIN, max.Y + T4_MARGIN));
        }
    }

    class Ballfield_topographer
    {
        private readonly T4 _t4;

        private const double T4_MARGIN = 1.0;

        public Point2F Min
        {
            get { return new Point2F(_t4.Rect.Xmin + T4_MARGIN, _t4.Rect.Ymin + T4_MARGIN); }
        }

        public Point2F Max
        {
            get { return new Point2F(_t4.Rect.Xmax - T4_MARGIN, _t4.Rect.Ymax - T4_MARGIN); }
        }

        private List<Circle2F> find_intersecting_balls(Line2F line)
        {
            T4_rect rect = new T4_rect(Math.Min(line.p1.X, line.p2.X),
                                       Math.Min(line.p1.Y, line.p2.Y),
                                       Math.Max(line.p1.X, line.p2.X),
                                       Math.Max(line.p1.Y, line.p2.Y));

            List<Circle2F> balls = new List<Circle2F>();

            // since objects in t4 are generic, convert convert rects backs to balls
            foreach (T4_rect ballrect in _t4.Get_colliding_obj_rects(rect))
                balls.Add(new Circle2F(new Point2F(ballrect.Xc, ballrect.Yc), ballrect.W / 2));

            return balls;
        }

        // we are collecting all the intersections and tracking the list of balls we're inside
        // at any given point. If list becomes empty, we can't shortcut
        public bool Is_line_inside_region(Line2F line, double tolerance)
        {
            Point2F a = line.p1;
            Point2F b = line.p2;

            SortedList<double, List<Circle2F>> intersections = new SortedList<double, List<Circle2F>>();
            List<Circle2F> running_balls = new List<Circle2F>();

            foreach (Circle2F ball in find_intersecting_balls(line))
            {
                Line2F insects = ball.LineIntersect(line, tolerance);

                if (insects.p1.IsUndefined && insects.p2.IsUndefined)
                {
                    // no intersections: check if whole path lay inside the circle
                    if (a.DistanceTo(ball.Center) < ball.Radius + tolerance
                        && b.DistanceTo(ball.Center) < ball.Radius + tolerance)
                        return true;
                }
                else if (insects.p1.IsUndefined || insects.p2.IsUndefined)
                {
                    // single intersection. one of the path ends must be inside the circle, otherwise it is a tangent case
                    // and should be ignored
                    if (a.DistanceTo(ball.Center) < ball.Radius + tolerance)
                    {
                        running_balls.Add(ball);
                    }
                    else if (b.DistanceTo(ball.Center) < ball.Radius + tolerance)
                    {
                        ;
                    }
                    else
                    {
                        continue;
                    }

                    Point2F c = insects.p1.IsUndefined ? insects.p2 : insects.p1;
                    double d = c.DistanceTo(a);
                    if (!intersections.ContainsKey(d))
                        intersections.Add(d, new List<Circle2F>());
                    intersections[d].Add(ball);
                }
                else
                {
                    // double intersection
                    double d = insects.p1.DistanceTo(a);
                    if (!intersections.ContainsKey(d))
                        intersections.Add(d, new List<Circle2F>());
                    intersections[d].Add(ball);

                    d = insects.p2.DistanceTo(a);
                    if (!intersections.ContainsKey(d))
                        intersections.Add(d, new List<Circle2F>());
                    intersections[d].Add(ball);
                }
            }

            if (running_balls.Count == 0)
                return false;

            foreach (var ins in intersections)
            {
                foreach (Circle2F s in ins.Value)
                {
                    if (running_balls.Contains(s))
                        running_balls.Remove(s);
                    else
                        running_balls.Add(s);
                }

                if (running_balls.Count == 0 && (ins.Key + tolerance < a.DistanceTo(b)))
                    return false;
            }

            return true;
        }

        public List<T> Get_colliding_objects<T>(Point2F min, Point2F max)
        {
            T4_rect rect = new T4_rect(min.X, min.Y, max.X, max.Y);
            return _t4.Get_colliding_objects<T>(rect);
        }

        public void Add(Point2F center, double radius, object obj)
        {
            T4_rect rect = new T4_rect(center.X - radius,
                                       center.Y - radius,
                                       center.X + radius,
                                       center.Y + radius);
            _t4.Add(rect, obj);
        }

        public void Add(Circle2F ball, object obj)
        {
            this.Add(ball.Center, ball.Radius, obj);
        }

        public Ballfield_topographer(Point2F min, Point2F max)
        {
            _t4 = new T4(new T4_rect(min.X - 1, min.Y - 1, max.X + 1, max.Y + 1));
        }
    }
}
