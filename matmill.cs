using System;
using System.Collections.Generic;

using CamBam.CAD;
using CamBam.Geom;

using Voronoi2;

namespace Medial_demo
{
    class Pocket_generator
    {
        private const double VORONOI_MARGIN = 1.0;
        private const bool ANALIZE_INNER_INTERSECTIONS = false;

        private readonly Polyline _outline;
        private readonly Polyline[] _islands;

        private double _general_tolerance = 0.001;
        private double _sample_step = 0.1;

        public double General_tolerance                         { set { _general_tolerance = value; } }
        public double Sample_step                               { set { _sample_step = value; } }

        private Point3F p3f(Point2F pt)
        {
            return new Point3F(pt.X, pt.Y, 0);
        }

        private Point2F p2f(Point3F pt)
        {
            return new Point2F(pt.X, pt.Y);
        }

        private List<Point2F> sample_curve(Polyline p, double step)
        {
            // divide curve evenly. There is a bug in CamBam's divide by step routine (duplicate points), while 'divide to n equal segments' should work ok.
            // execution speed may be worse, but who cares
            double length = p.GetPerimeter();
            int nsegs = (int)Math.Max(Math.Ceiling(length / step), 1);

            List<Point2F> points = new List<Point2F>();
            foreach (Point3F pt in PointListUtils.CreatePointlistFromPolyline(p, nsegs).Points)
                points.Add((Point2F) pt);

            return points;
        }

        private bool is_line_inside_region(Line2F line, bool should_analize_inner_intersections)
        {
            if (!_outline.PointInPolyline(line.p1, _general_tolerance)) return false;     // p1 is outside of outer curve boundary
            if (!_outline.PointInPolyline(line.p2, _general_tolerance)) return false;  // p2 is outside of outer curve boundary
            if (should_analize_inner_intersections && _outline.LineIntersections(line, _general_tolerance).Length != 0) return false; // both endpoints are inside, but there are intersections, outer curve must be concave

            foreach (Polyline island in _islands)
            {
                if (island.PointInPolyline(line.p1, _general_tolerance)) return false;  // p1 is inside hole
                if (island.PointInPolyline(line.p2, _general_tolerance)) return false;  // p2 is inside hole
                if (should_analize_inner_intersections && island.LineIntersections(line, _general_tolerance).Length != 0) return false; // p1, p2 are outside hole, but there are intersections
            }
            return true;
        }

        private List<Line2F> get_mat_segments()
        {
            List<Point2F> plist = new List<Point2F>();

            plist.AddRange(sample_curve(_outline, _sample_step));
            foreach (Polyline p in _islands)
                plist.AddRange(sample_curve(p, _sample_step));

            Host.log("got {0} points", plist.Count);

            double[] xs = new double[plist.Count];
            double[] ys = new double[plist.Count];

            double min_x = double.MaxValue;
            double max_x = double.MinValue;
            double min_y = double.MaxValue;
            double max_y = double.MinValue;

            // HACK
            // There is a bug in Voronoi generator implementation. Sometimes it produces a completely crazy partitioning.
            // Looks like its overly sensitive to the first processed points, their count and location. If first stages
            // go ok, then everything comes nice. Beeing a Steven Fortune's algorithm, it process points by a moving sweep line.
            // Looks like the line is moving from the bottom to the top, thus sensitive points are the most bottom ones.
            // We try to cheat and move one of the most bottom points (there may be a lot, e.g. for rectange) a little
            // lower. Then generator initially will see just one point, do a right magic and continue with a sane result :-)
            // Let's always move a leftmost bottom point to be distinct.

            int hackpoint_idx = 0;

            for (int i = 0; i < plist.Count; i++)
            {
                xs[i] = plist[i].X;
                ys[i] = plist[i].Y;
                if (xs[i] < min_x) min_x = xs[i];
                if (xs[i] > max_x) max_x = xs[i];
                if (ys[i] > max_y) max_y = ys[i];

                if (ys[i] <= min_y)
                {
                    if (ys[i] < min_y)
                    {
                        min_y = ys[i];
                        hackpoint_idx = i;  // stricly less, it's a new hackpoint for sure
                    }
                    else
                    {
                        if (xs[i] < xs[hackpoint_idx])  // it's a new hackpoint if more lefty
                            hackpoint_idx = i;
                    }
                }
            }

            ys[hackpoint_idx] -= _general_tolerance;

            min_x -= VORONOI_MARGIN;
            max_x += VORONOI_MARGIN;
            min_y -= VORONOI_MARGIN;
            max_y += VORONOI_MARGIN;

            List<GraphEdge> edges = new Voronoi(_general_tolerance).generateVoronoi(xs, ys, min_x, max_x, min_y, max_y);

            Host.log("voronoi partitioning completed. got {0} edges", edges.Count);

            List<Line2F> inner_segments = new List<Line2F>();

            foreach (GraphEdge e in edges)
            {
                Line2F seg = new Line2F(e.x1, e.y1, e.x2, e.y2);

                if (seg.Length() < _general_tolerance) continue;    // extra small segment, discard
                if (! is_line_inside_region(seg, ANALIZE_INNER_INTERSECTIONS)) continue;
                inner_segments.Add(seg);
            }

            Host.log("got {0} inner segments", inner_segments.Count);

            return inner_segments;
        }

        private void attach_segments(Branch me, Segpool pool)
        {
            Point2F running_end = (Point2F)me.Curve.LastPoint;
            List<Point2F> followers;

            while (true)
            {
                followers = pool.Pull_follow_points(running_end);

                if (followers.Count != 1)
                    break;

                running_end = followers[0];
                me.Curve.Add(p3f(running_end));   // continuation
            }

            if (followers.Count == 0) return; // end of branch, go out

            foreach (Point2F pt in followers)
            {
                Branch b = new Branch(me);
                b.Curve.Add(p3f(running_end));
                b.Curve.Add(p3f(pt));
                attach_segments(b, pool);

                me.Children.Add(b);
            }
            // prefer a shortest branch
            me.Children.Sort((a, b) => a.Deep_distance().CompareTo(b.Deep_distance()));
        }

        private Branch build_tree(List<Line2F> segments)
        {
            if (segments.Count == 0)
                return null;

            Segpool pool = new Segpool(segments.Count, _general_tolerance);

            Host.log("analyzing segments");

            foreach (Line2F line in segments)
            {
                pool.Add(line, false);
                pool.Add(line, true);
            }

            Host.log("done analyzing segments");
            Host.log("got {0} hashes", pool.N_hashes);

            Branch root = new Branch(null);
            root.Curve.Add(p3f(segments[0].p1));    // startpoint would be just the first segment (ok for demo)
            attach_segments(root, pool);
            return root;
        }

        public List<Polyline>run()
        {
            List<Line2F> mat_lines = get_mat_segments();

            Host.log("building tree");
            Branch root = build_tree(mat_lines);
            if (root == null)
            {
                Host.warn("failed to build tree");
                return null;
            }

            List<Branch> traverse = root.Df_traverse();
            List<Polyline> medial_axis = new List<Polyline>();

            foreach (Branch b in traverse)
                medial_axis.Add(b.Curve);

            return medial_axis;
        }

        public Pocket_generator(Polyline outline, Polyline[] islands)
        {
            _outline = outline;
            _islands = islands;

            Point3F min = Point3F.Undefined;
            Point3F max = Point3F.Undefined;

            _outline.GetExtrema(ref min, ref max);
        }
    }
}
