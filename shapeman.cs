using System;
using System.Collections.Generic;

using CamBam.CAD;
using CamBam.Geom;

using Voronoi2;

using Tree4;

namespace Matmill
{
    class Shapeman
    {
        private const double VORONOI_MARGIN = 1.0;
        private const bool ANALIZE_INNER_INTERSECTIONS = false;

        private readonly Polyline _outline;
        private readonly Polyline[] _islands;
        private readonly T4 _t4;

        public T4_rect Bbox
        {
            get { return _t4.Rect; }
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

        private bool is_line_inside_region(Line2F line, bool should_analize_inner_intersections, double tolerance)
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

        public List<Line2F> Get_medial_axis_segments(double sample_step, double general_tolerance)
        {
            List<Point2F> samples = Get_samples(sample_step);

            Logger.log("got {0} points", samples.Count);

            double[] xs = new double[samples.Count + 1];
            double[] ys = new double[samples.Count + 1];

            double min_x = double.MaxValue;
            double max_x = double.MinValue;
            double min_y = double.MaxValue;
            double max_y = double.MinValue;

            // HACK
            // There is a bug in Voronoi generator implementation. Sometimes it produces a completely crazy partitioning.
            // Looks like its overly sensitive to the first processed points, their count and location. If first stages
            // go ok, then everything comes nice. Beeing a Steven Fortune's algorithm, it process points by a moving sweep line.
            // Looks like the line is moving from the bottom to the top, thus sensitive points are the most bottom ones.
            // We try to cheat and add one more point so it would be the single most bottom point.
            // Then generator initially will see just one point, do a right magic and continue with a sane result :-)
            // We place this initial point under the lefmost bottom point at the sufficient distance,
            // then these two points will form a separate Voronoi cite not influencing the remaining partition.
            // Sufficient distance is defined as region width / 2 for now.

            int lb_idx = 0;

            for (int i = 0; i < samples.Count; i++)
            {
                xs[i] = samples[i].X;
                ys[i] = samples[i].Y;
                if (xs[i] < min_x) min_x = xs[i];
                if (xs[i] > max_x) max_x = xs[i];
                if (ys[i] > max_y) max_y = ys[i];

                if (ys[i] <= min_y)
                {
                    if (ys[i] < min_y)
                    {
                        min_y = ys[i];
                        lb_idx = i;  // stricly less, it's a new leftmost bottom for sure
                    }
                    else
                    {
                        if (xs[i] < xs[lb_idx])  // it's a new leftmost bottom if more lefty
                            lb_idx = i;
                    }
                }
            }

            double width = max_x - min_x;
            xs[samples.Count] = xs[lb_idx];
            ys[samples.Count] = ys[lb_idx] - width / 2;

            min_x -= VORONOI_MARGIN;
            max_x += VORONOI_MARGIN;
            min_y -= VORONOI_MARGIN + width / 2;
            max_y += VORONOI_MARGIN;

            List<GraphEdge> edges = new Voronoi(general_tolerance).generateVoronoi(xs, ys, min_x, max_x, min_y, max_y);

            Logger.log("voronoi partitioning completed. got {0} edges", edges.Count);

            List<Line2F> inner_segments = new List<Line2F>();

            foreach (GraphEdge e in edges)
            {
                Line2F seg = new Line2F(e.x1, e.y1, e.x2, e.y2);
                if (seg.Length() < double.Epsilon) continue;    // extra small segment, discard
                if (! is_line_inside_region(seg, ANALIZE_INNER_INTERSECTIONS, general_tolerance)) continue;
                inner_segments.Add(seg);
            }

            return inner_segments;
        }

        public double Get_dist_to_wall(Point2F pt)
        {
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
            return is_line_inside_region(line, true, tolerance);
        }

        public List<Point2F> Get_samples(double sample_step)
        {
            List<Point2F> plist = new List<Point2F>();

            plist.AddRange(sample_curve(_outline, sample_step));
                foreach (Polyline p in _islands)
                    plist.AddRange(sample_curve(p, sample_step));

            return plist;
        }

        public Shapeman(Polyline outline, Polyline[] islands)
        {
            _outline = outline;
            _islands = islands;

            Point3F min = Point3F.Undefined;
            Point3F max = Point3F.Undefined;

            _outline.GetExtrema(ref min, ref max);

            _t4 = new T4(new T4_rect(min.X - 1, min.Y - 1, max.X + 1, max.Y + 1));

            insert_in_t4(_outline);
            foreach (Polyline island in _islands)
                insert_in_t4(island);
        }
    }
}
