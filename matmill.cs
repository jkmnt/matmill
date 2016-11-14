using System;
using System.Collections;
using System.Collections.Generic;

using CamBam.UI;
using CamBam.CAD;
using CamBam.Geom;

using Tree4;
using Voronoi2;

namespace Matmill
{
    class Curve
    {
        // XXX: empty curve shouldn't be accessed
        private List<Point2F> _points = new List<Point2F>();
        private List<double> _seg_offsets = new List<double>();
        private double _len = 0;

        public List<Point2F> Points { get {return _points;}}
        public Point2F Start { get {return _points[0];}}
        public Point2F End { get {return _points[_points.Count - 1];}}
        public double Length { get {return _len;}}

        public void Add(Point2F pt)
        {
            if (_points.Count != 0)
            {
                double seg_len = End.DistanceTo(pt);
                _seg_offsets.Add(_len);
                _len += seg_len;
            }
            _points.Add(pt);
        }

        public Point2F Get_parametric_pt(double u)
        {
            if (_points.Count < 2)
                return _points[0];

            double offset = u * _len;
            // TODO: use binary search
            int seg;
            for (seg = 1; seg < _seg_offsets.Count; seg++)
            {
                if (_seg_offsets[seg] > offset)
                    break;
            }

            seg -= 1;
            offset -= _seg_offsets[seg];

            Point2F p1 = _points[seg];
            Point2F p2 = _points[seg + 1];
            double dist = p2.DistanceTo(p1);
            double x = p1.X + offset / dist * (p2.X - p1.X);
			double y = p1.Y + offset / dist * (p2.Y - p1.Y);

            return new Point2F(x, y);
        }

        public Curve()
        {
        }
    }

    class Slice
    {
        // TODO: ball or arc: choose only one representation
        private Circle2F _ball;
        private Arc2F _arc;
        private double _max_engagement;
        private List<Arc2F> _segments = new List<Arc2F>();
        private Slice _prev_slice;
        private bool _is_undefined = true;

        public Circle2F Ball { get { return _ball; } }
        public Arc2F Arc { get { return _arc; } }
        public bool Is_undefined { get { return _is_undefined; } }

        public Point2F Center { get { return _ball.Center; } }
        public double Radius { get { return _ball.Radius; } }
        public Slice Prev { get { return _prev_slice; } }
        public double Max_engagement { get { return _max_engagement; } }
        public List<Arc2F> Segments { get { return _segments; } }

        static public double Calc_max_engagement(Point2F center, double radius, Slice prev_slice)
        {
            double delta_s = Point2F.Distance(center, prev_slice.Center);
            double delta_r = radius - prev_slice.Radius;
            return delta_s + delta_r;
        }

        static public double Calc_lens(Point2F center_0, double radius_0, Point2F center_1, double radius_1)
        {
            // http://mathworld.wolfram.com/Lens.html
            // area of inner lens between two circles
            double d = center_0.DistanceTo(center_1);
            //double R = prev_slice.Radius + cutter_r;
            double R = radius_0;
            double r = radius_1;

            // no lens in this cases, return 0.
            if (d == 0)
            if (R + r <= d) return 0;
            if (R >= r && (R >= r + d)) return 0;
            if (r >= R && (r >= R + d)) return 0;

            double A_lens =    r * r * Math.Acos((d*d + r*r - R*R) / 2.0 / d / r)
                             + R * R * Math.Acos((d*d + R*R - r*r) / 2.0 / d / R)
                             - Math.Sqrt((-d + r + R)*(d + r - R)*(d - r + R)*(d + r +R)) / 2.0;
            return A_lens;
        }

        public double Calc_mrr(double cutter_r)
        {
            double R = _prev_slice.Radius + cutter_r;
            double r = _ball.Radius;

            double A_lens = Calc_lens(_prev_slice.Center, _prev_slice.Radius, _ball.Center, r);
            if (A_lens == 0) return 0;

            return Math.PI * r * r - A_lens;
        }

        // temporary lightwidth slice
        public Slice(Slice prev_slice, Point2F center, double radius, RotationDirection dir)
        {
            _prev_slice = prev_slice;
            _ball = new Circle2F(center, radius);
            _max_engagement = Slice.Calc_max_engagement(center, radius, _prev_slice);

            Line2F insects = _prev_slice.Ball.CircleIntersect(_ball);

            if (insects.p1.IsUndefined || insects.p2.IsUndefined) return;

            _arc = new Arc2F(_ball.Center, insects.p1, insects.p2, dir);

            if (! _arc.VectorInsideArc(new Vector2F(_prev_slice.Center, _ball.Center)))
                _arc = new Arc2F(_ball.Center, insects.p2, insects.p1, dir);

            _is_undefined = false;
        }

        public void Refine(List<Slice> colliding_slices, double end_clearance)
        {
            double clearance = end_clearance;

            // check if arc is small. refining is worthless in this case
            // criterion for smallness: there should be at least 4 segments with chord = clearance, plus
            // one segment to space ends far enough. A pentagon with a 5 segments with edge length = clearance
            // will define the min radius of circumscribed circle. clearance = 2 * R * sin (Pi / 5),
            // R = clearance / 2 / sin (Pi / 5)

            double r_min = clearance / 2 / Math.Sin(Math.PI / 5.0);
            if (_arc.Radius <= r_min)
                return;

            if (colliding_slices.Contains(this))
            {
                // XXX: assert here
                Host.err("contains ME !");
                return;
            }

            // now apply the colliding slices. to keep things simple and robust, we apply just one slice - the one who trims
            // us most (removed length of arc is greatest).

            // end clearance adjustment:
            // to guarantee the cutter will never hit the unmilled area while rapiding between segments,
            // arc will always have original ends, trimming will happen in the middle only.
            // to prevent the cutter from milling extra small end segments and minimize numeric errors at small tangents,
            // original ends would always stick at least for a clearance (chordal) length.
            // i.e. if the point of intersection of arc and colliding circle is closer than clearance to the end,
            // it is moved to clearance distance.

            // there is a two cases of intersecting circles: with single intersection and with a double intersection.
            // double intersections splits arc to three pieces (length of the middle part is the measure),
            // single intesections splits arc in two (the part inside the circle is removed, its length is the measure).
            // in both cases the intersection points are subject to "end clearance adjustment".
            // single intersections are transformed to the double intersections, second point being one of the end clearances.

            // TODO: calculate clearance point the right way, with math :-)
            Line2F c1_insects = _arc.CircleIntersect(new Circle2F(_arc.P1, clearance));
            Line2F c2_insects = _arc.CircleIntersect(new Circle2F(_arc.P2, clearance));
            Point2F c1 = c1_insects.p1.IsUndefined ? c1_insects.p2 : c1_insects.p1;
            Point2F c2 = c2_insects.p1.IsUndefined ? c2_insects.p2 : c2_insects.p1;

            Line2F max_secant = new Line2F();
            double max_sweep = 0;

            foreach (Slice s in colliding_slices)
            {
                if (s == _prev_slice) continue;  // no reason to process it
                Line2F secant = _arc.CircleIntersect(s.Ball);

                if (secant.p1.IsUndefined && secant.p2.IsUndefined) continue;

                if (secant.p1.IsUndefined || secant.p2.IsUndefined)
                {
                    // single intersection
                    Point2F splitpt = secant.p1.IsUndefined ? secant.p2 : secant.p1;
                    if (_arc.P1.DistanceTo(s.Ball.Center) < _arc.P2.DistanceTo(s.Ball.Center))
                    {
                        if (splitpt.DistanceTo(_arc.P1) < clearance)
                            continue;  // nothing to remove
                        else if (splitpt.DistanceTo(_arc.P2) < clearance)
                            secant = new Line2F(c1, c2);
                        else
                            secant = new Line2F(c1, splitpt);
                    }
                    else
                    {
                        // remove second segment
                        if (splitpt.DistanceTo(_arc.P2) < clearance)
                            continue;
                        else if (splitpt.DistanceTo(_arc.P1) < clearance)
                            secant = new Line2F(c1, c2);
                        else
                            secant = new Line2F(splitpt, c2);
                    }
                }
                else
                {
                    // double intersection
                    if (secant.p1.DistanceTo(_arc.P1) < clearance)
                        secant.p1 = c1;
                    else if (secant.p1.DistanceTo(_arc.P2) < clearance)
                        secant.p1 = c2;

                    if (secant.p2.DistanceTo(_arc.P1) < clearance)
                        secant.p2 = c1;
                    else if (secant.p2.DistanceTo(_arc.P2) < clearance)
                        secant.p2 = c2;
                }

                if (secant.p1.DistanceTo(secant.p2) < clearance * 2) // segment is too short, ignore it
                    continue;

                // sort insects by sweep (already sorted for single, may be unsorted for the double)
                Vector2F v_p1 = new Vector2F(_arc.Center, _arc.P1);
                Vector2F v_ins1 = new Vector2F(_arc.Center, secant.p1);
                Vector2F v_ins2 = new Vector2F(_arc.Center, secant.p2);

                double sweep = Sweep_comparer.calc_angle(v_ins1, v_ins2, _arc.Direction);

                if (  Sweep_comparer.calc_angle(v_p1, v_ins1, _arc.Direction) > Sweep_comparer.calc_angle(v_p1, v_ins2, _arc.Direction))
                {
                    secant = new Line2F(secant.p2, secant.p1);
                    sweep = 2.0 * Math.PI - sweep;
                }

                if (sweep > max_sweep)
                {
                    // ok, a last check - removed arc midpoint should be inside the colliding circle
                    Arc2F arc = new Arc2F(_arc.Center, secant.p1, secant.p2, _arc.Direction);
                    if (arc.Midpoint.DistanceTo(s.Ball.Center) < s.Ball.Radius)
                    {
                        max_sweep = sweep;
                        max_secant = secant;
                    }
                }
            }

            if (max_sweep == 0)
            {
                _segments.Add(_arc);
            }
            else
            {
                Arc2F start = new Arc2F(_arc.Center, _arc.P1, max_secant.p1, _arc.Direction);
                Arc2F removed = new Arc2F(_arc.Center, max_secant.p1, max_secant.p2, _arc.Direction);
                Arc2F end = new Arc2F(_arc.Center, max_secant.p2, _arc.P2, _arc.Direction);

                _segments.Add(start);
                _segments.Add(end);

                // recalculate engagement if base engagement is no longer valid (midpoint vanished with the removed middle segment).
                // this engagement is 'virtual' and averaged with base to reduce stress on cutter abruptly entering the wall
                // TODO: is it really needed ?
                // TOD: should we apply opposite derating if base engagement is valid ?
                if (removed.VectorInsideArc(new Vector2F(_arc.Center, _arc.Midpoint)))
                {
                    double e0 = _prev_slice.Center.DistanceTo(max_secant.p1) - _prev_slice.Radius;
                    double e1 = _prev_slice.Center.DistanceTo(max_secant.p2) - _prev_slice.Radius;

                    if (false)
                    {
                        _max_engagement += Math.Max(e0, e1);
                        _max_engagement /= 2.0;
                    }
                    else
                    {
                        _max_engagement = Math.Max(e0, e1);
                    }
                }
            }
        }

        public Slice(Point2F center, double radius, RotationDirection dir)
        {
            _ball = new Circle2F(center, radius);
            _max_engagement = 0;
            // XXX: hack, just for now
            _arc = new Arc2F(center, radius, 0, 359);
            Arc2F arc0 = new Arc2F(center, radius, 0, 120);
            Arc2F arc1 = new Arc2F(center, radius, 120, 120);
            Arc2F arc2 = new Arc2F(center, radius, 240, 120);
            _segments.Add(arc0);
            _segments.Add(arc1);
            _segments.Add(arc2);
            _is_undefined = false;
        }
    }

    class Branch
    {
        public delegate void Visitor(Branch b);

        public readonly Curve Curve = new Curve();
        public readonly Branch Parent = null;
        public readonly List<Branch> Children = new List<Branch>();
        public readonly List<Slice> Slices = new List<Slice>();
        public Entity Entry = null;
        public string Debug = "";

        public bool Is_leaf { get { return Children.Count == 0; } }

        public List<Branch> Df_traverse()  //
        {
            List<Branch> result = new List<Branch>();
            result.Add(this);
            foreach (Branch b in Children)
                result.AddRange(b.Df_traverse());
            return result;
        }

        public void Df_visit(Visitor v)
        {
            v(this);
            foreach (Branch b in Children)
                b.Df_visit(v);
        }

        public double Deep_distance()
        {
            double dist = Curve.Length;
            foreach (Branch b in Children)
                dist += b.Deep_distance();

            return dist;
        }

        public List<Branch> Get_parents()
        {
            List<Branch> parents = new List<Branch>();
            for (Branch p = Parent; p != null; p = p.Parent)
            {
                parents.Add(p);
            }
            return parents;
        }

        // Get all the slices blocking path (meet first) while traveling up the branch
        // (and followind neighbour downstream subbranches)
        public List<Slice> Get_upstream_roadblocks()
        {
            List<Slice> candidates = new List<Slice>();

            for (Branch visited = this; visited.Parent != null; visited = visited.Parent)
            {
                Branch upstream = visited.Parent;

                if (upstream.Children.Count != 0)
                {
                    foreach (Branch child in upstream.Children)
                    {
                        // except the path we're walking now
                        if (child != visited)
                            candidates.AddRange(child.Get_downstream_roadblocks());
                    }
                }

                if (upstream.Slices.Count != 0)
                {
                    candidates.Add(upstream.Slices[upstream.Slices.Count - 1]);
                    break;
                }
            }

            return candidates;
        }

        // Get all the slices blocking path (meet first) while traveling down the branch
        // and next subbranches
        public List<Slice> Get_downstream_roadblocks()
        {
            List<Slice> candidates = new List<Slice>();

            if (Slices.Count != 0)
            {
                candidates.Add(Slices[0]);
            }
            else
            {
                foreach (Branch c in Children)
                    candidates.AddRange(c.Get_downstream_roadblocks());
            }

            return candidates;
        }

        public Branch(Branch parent)
        {
            Parent = parent;
        }
    }

    class Sweep_comparer : IComparer
    {
        private readonly Point2F _origin;
        private readonly Point2F _center;
        private readonly RotationDirection _dir;
        private readonly Vector2F _start_vector;

        static public double calc_angle(Vector2F v0, Vector2F v1, RotationDirection dir)
        {
            double angle = Math.Atan2(Vector2F.Determinant(v0, v1), Vector2F.DotProduct(v0, v1));
            if (angle < 0)
                angle += 2.0 * Math.PI;
            return (dir == RotationDirection.CCW) ? angle : (2.0 * Math.PI - angle);
        }

        private double angle_to_start_vector(Point2F p)
        {
            return calc_angle(_start_vector, new Vector2F(_center, p), _dir);
        }

        public Sweep_comparer(Point2F origin, Point2F center, RotationDirection dir)
        {
            _origin = origin;
            _center = center;
            _dir = dir;
            _start_vector = new Vector2F(center, origin);
        }

        public int Compare(object a, object b)
        {
            return angle_to_start_vector((Point2F)a).CompareTo(angle_to_start_vector((Point2F)b));
        }

    }

    class Pocket_generator
    {
        private const double GENERAL_TOLERANCE = 0.001;
        private const double VORONOI_MARGIN = 1.0;
        private const bool ANALIZE_INNER_INTERSECTIONS = false;
        private const double ENGAGEMENT_TOLERANCE_PERCENTAGE = 0.001;  // 0.1 %


        private readonly Region _reg;
        private readonly T4 _reg_t4;

        private double _cutter_r = 1.5;
        private double _max_engagement = 3.0 * 0.4;
        private double _min_engagement = 3.0 * 0.1;

        private Point2F _toolpoint = Point2F.Undefined;

        private RotationDirection _dir = RotationDirection.CW;

        public double Cutter_d        {set { _cutter_r = value / 2.0;}}
        public double Max_engagement  {set { _max_engagement = value; } }
        public double Min_engagement  {set { _min_engagement = value; } }
        public RotationDirection Mill_direction  {set { _dir = value; } }

        private Point3F point(Point2F p2)
        {
            return new Point3F(p2.X, p2.Y, 0);
        }

        private Point2F point(Point3F p3)
        {
            return new Point2F(p3.X, p3.Y);
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

        private List<Line2F> get_mat_segments()
        {
            List<Point2F> plist = new List<Point2F>();

            plist.AddRange(sample_curve(this._reg.OuterCurve, _cutter_r / 10));
            foreach (Polyline p in this._reg.HoleCurves)
                plist.AddRange(sample_curve(p, _cutter_r / 10));

            Host.log("Got {0} points", plist.Count);

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

            ys[hackpoint_idx] -= GENERAL_TOLERANCE;

            min_x -= VORONOI_MARGIN;
            max_x += VORONOI_MARGIN;
            min_y -= VORONOI_MARGIN;
            max_y += VORONOI_MARGIN;

            List<GraphEdge> edges = new Voronoi(GENERAL_TOLERANCE).generateVoronoi(xs, ys, min_x, max_x, min_y, max_y);

            Host.log("voroning completed. Got {0} edges", edges.Count);

            List<Line2F> inner_segments = new List<Line2F>();

            foreach (GraphEdge e in edges)
            {
                Line2F seg = new Line2F(e.x1, e.y1, e.x2, e.y2);

                if (seg.Length() < GENERAL_TOLERANCE) continue;    // extra small segment, discard
                if (!_reg.OuterCurve.PointInPolyline(seg.p1, GENERAL_TOLERANCE)) continue;  // p1 is outside of outer curve boundary
                if (!_reg.OuterCurve.PointInPolyline(seg.p2, GENERAL_TOLERANCE)) continue;  // p2 is outside of outer curve boundary
                if (ANALIZE_INNER_INTERSECTIONS && _reg.OuterCurve.LineIntersections(seg, GENERAL_TOLERANCE).Length != 0) continue; // both endpoints are inside, but there are intersections, outer curve must be concave

                bool should_add = true;
                foreach (Polyline hole in this._reg.HoleCurves)
                {
                    if (   hole.PointInPolyline(seg.p1, GENERAL_TOLERANCE)                      // p1 is inside hole
                        || hole.PointInPolyline(seg.p2, GENERAL_TOLERANCE)                      // p2 is inside hole
                        || (ANALIZE_INNER_INTERSECTIONS && hole.LineIntersections(seg, GENERAL_TOLERANCE).Length != 0))          // p1, p2 are outside hole, but there are intersections
                    {
                        should_add = false;
                        break;
                    }
                }

                if (should_add)
                    inner_segments.Add(seg);
            }

            Host.log("Got {0} inner segments", inner_segments.Count);

            return inner_segments;
        }

        private double get_mic_radius(Point2F pt)
        {
            double radius = double.MaxValue;
            foreach(object item in _reg_t4.Get_nearest_objects(pt.X, pt.Y))
            {
                double dist = 0;
                if (item is Line2F)
                    ((Line2F)item).NearestPoint(pt, ref dist);
                else
                    ((Arc2F)item).NearestPoint(pt, ref dist);
                if (dist < radius)
                    radius = dist;
            }

            // account for margin just in one subrtract. Nice !
            if (false)
            {
                radius -= _cutter_r;
            }

            return radius;
        }

        private Slice find_prev_slice(Branch branch, Point2F pt, double radius, T4 ready_slices)
        {
            Slice prev_slice = null;

            double min_engage = double.MaxValue;

            List<Slice> candidates = branch.Get_upstream_roadblocks();
            foreach (Slice candidate in candidates)
            {
                Slice s = new Slice(candidate, pt, radius, _dir);
                s.Refine(find_colliding_slices(s, ready_slices), _cutter_r);

                //double slice_engage = Slice.Calc_max_engagement(pt, radius, candidate);
                double slice_engage = s.Max_engagement;
                if (slice_engage > _max_engagement)
                    continue;

                //double mrr = Slice.Calc_mrr(pt, radius, candidate, _cutter_r);
                //double dist = pt.DistanceTo(candidate.Center);

                if (prev_slice == null || slice_engage < min_engage)
                {
                    min_engage = slice_engage;
                    prev_slice = candidate;
                }
            }

            return prev_slice;
        }

        private Slice find_nearest_slice(Branch branch, Point2F pt, double radius, T4 ready_slices)
        {
            Slice prev_slice = null;

            double min_dist = double.MaxValue;

            List<Slice> candidates = branch.Get_upstream_roadblocks();
            foreach (Slice candidate in candidates)
            {
                double dist = pt.DistanceTo(candidate.Center);
                if (prev_slice == null || dist < min_dist)
                {
                    min_dist = dist;
                    prev_slice = candidate;
                }
            }

            return prev_slice;
        }

        private List<Slice> find_colliding_slices(Slice s, T4 ready_slices)
        {
            Point2F min = Point2F.Undefined;
            Point2F max = Point2F.Undefined;
            s.Arc.GetExtrema(ref min, ref max);
            T4_rect rect = new T4_rect(min.X, min.Y, max.X, max.Y);
            List<Slice> result = new List<Slice>();
//            foreach (object obj in ready_slices.Get_colliding_objects(ready_slices.Rect))
//                result.Add((Slice)obj);
            // TODO: is there a way to do it without repacking ?
            foreach (object obj in ready_slices.Get_colliding_objects(rect))
                result.Add((Slice)obj);
            return result;
        }

        // find least common ancestor for the both branches
        Entity switch_branch(Slice dst, Slice src, T4 ready_slices)
        {
            List<Slice> path = new List<Slice>();

            // continuation
            if (dst.Prev == src)
            {
                return (Entity)new Line(src.Segments[src.Segments.Count - 1].P2, dst.Segments[0].P1);
            }
            else
            {

                List<Slice> src_ancestry = new List<Slice>();
                List<Slice> dst_ancestry = new List<Slice>();

                for (Slice s = src.Prev; s != null; s = s.Prev)
                    src_ancestry.Insert(0, s);

                for (Slice s = dst.Prev; s != null; s = s.Prev)
                    dst_ancestry.Insert(0, s);

                int lca;
                for (lca = 0; lca < Math.Min(src_ancestry.Count, dst_ancestry.Count); lca++)
                {
                    if (src_ancestry[lca] != dst_ancestry[lca])
                        break;
                }

                // now lca contains the lca of branches
                // collect path up from src to lca and down to dst
                for (int i = src_ancestry.Count - 1; i > lca; i--)
                    path.Add(src_ancestry[i]);

                for (int i = lca; i < dst_ancestry.Count - 1; i++)
                    path.Add(dst_ancestry[i]);

                // trace path
                Polyline p = new Polyline();
                Point2F current = src.Segments[src.Segments.Count - 1].P2;
                Point2F end = dst.Segments[0].P1;

                p.Add(point(current));

                // follow the path, while looking for a shortcut to reduce travel time
                // TODO: skip parts of path to reduce travel even more
                for (int i = 0; i < path.Count; i++)
                {
                    Slice s = path[i];
                    if (may_shortcut(current, end, ready_slices))
                        break;

                    current = s.Center;
                    p.Add(point(current));
                }

                p.Add(point(end));

                return (Entity)p;
            }
        }

        private void roll(Branch branch, T4 ready_slices, ref Slice last_slice)
        {
            Slice prev_slice = null;

            // non-initial slice
            if (branch.Parent != null)
            {
                Point2F pt = branch.Curve.Start;
                prev_slice = find_prev_slice(branch, pt, get_mic_radius(pt), ready_slices);

                if (prev_slice == null)
                {
                    Host.log("Failed to attach branch");
                    return;
                }
            }
            else
            {
                // top branch should always had a big circle at pt[0] !
                //XXX: verify it !
                Point2F pt = branch.Curve.Start;
                double radius = get_mic_radius(pt);

                Slice s = new Slice(pt, radius, _dir);
                branch.Slices.Add(s);
                // XXX: for now
                insert_in_t4(ready_slices, s);
                prev_slice = s;
            }

            double left = 0;
            while (true)
            {
                Slice s;

                double right = 1.0;

                while (true)
                {
                    double mid = (left + right) / 2;

                    Point2F pt = branch.Curve.Get_parametric_pt(mid);

                    double radius = get_mic_radius(pt);
                    s = new Slice(prev_slice, pt, radius, _dir);
                    if (s.Is_undefined) // undefined means engagement is too big, slices do not intersect at all
                    {
                        right = mid;
                    }
                    else
                    {
                        s.Refine(find_colliding_slices(s, ready_slices), _cutter_r);

                        // prefer slight undershoot withing the engagement tolerance
                        if (s.Max_engagement > _max_engagement)
                        {
                            right = mid;
                        }
                        else if (Math.Abs(s.Max_engagement - _max_engagement) / _max_engagement < ENGAGEMENT_TOLERANCE_PERCENTAGE)
                        {
                            left = mid;
                            break;
                        }
                        else
                        {
                            left = mid;
                        }
                    }

                    Point2F other = branch.Curve.Get_parametric_pt(left == mid ? right : left);

                    if (pt.DistanceTo(other) < GENERAL_TOLERANCE)
                    {
                        left = mid;
                        break;
                    }
                }

                if (s.Radius < _cutter_r)
                    return;

                double err = (s.Max_engagement - _max_engagement) / _max_engagement;

                if (err > ENGAGEMENT_TOLERANCE_PERCENTAGE * 10)
                {
                    Host.err("Failed to create slice within stepover limit. Stopping slicing the branch.");
                    return;
                }

                if (s.Max_engagement < _min_engagement)
                    return;

                if (branch.Slices.Count == 0 && last_slice != null)
                {
                    branch.Entry = switch_branch(s, last_slice, ready_slices);
                }

                branch.Slices.Add(s);
                insert_in_t4(ready_slices, s);
                prev_slice = s;
                last_slice = s;
            }
        }

        private void attach_segments(Branch me, Segpool pool)
        {
            Point2F running_end = me.Curve.End;
            List<Point2F> followers;

            while (true)
            {
                followers = pool.Pull_follow_points(running_end);

                if (followers.Count != 1)
                    break;

                running_end = followers[0];
                me.Curve.Add(running_end);   // continuation
            }

            if (followers.Count == 0) return; // end of branch, go out

            foreach (Point2F pt in followers)
            {
                Branch b = new Branch(me);
                b.Curve.Add(running_end);
                b.Curve.Add(pt);
                attach_segments(b, pool);

                me.Children.Add(b);
            }
            // prefer a shortest branch
            me.Children.Sort((a, b) => a.Deep_distance().CompareTo(b.Deep_distance()));
        }

        private Branch build_tree(List<Line2F> segments)
        {
            // determine the start segment - the one with the largest mic
            double largest_radius = double.MinValue;
            Point2F start_pt = Point2F.Undefined;

            Segpool pool = new Segpool(segments.Count, GENERAL_TOLERANCE);

            // XXX: p2 is not analyzed
            Host.log("analyzing segments");
            foreach (Line2F line in segments)
            {
                double r = get_mic_radius(line.p1);
                if (r < _cutter_r)
                {
                    // strange. maybe this segment is unmillable
                    r = get_mic_radius(line.p2);
                    if (r < _cutter_r)  // unmillable
                        continue;
                }

                if (r > largest_radius)
                {
                    largest_radius = r;
                    start_pt = line.p1;
                }

                pool.Add(line);
            }

            Host.log("done analyzing segments");
            Host.log("got {0} hashes", pool.N_hashes);

            // XXX: startpoint may be undefined, fix it later

            // craft new artifical start poly
            Branch root = new Branch(null);
            root.Curve.Add(start_pt);
            attach_segments(root, pool);

            return root;
        }

        private void insert_in_t4(T4 t4, Slice slice)
        {
            Circle2F circle = slice.Ball;
            Point2F center = circle.Center;
            double radius = circle.Radius;
            t4.Add(new T4_rect(center.X - radius, center.Y - radius, center.X + radius, center.Y + radius), slice);
        }

        private void insert_in_t4(T4 t4, Polyline p)
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
                    // XXX: assert here ?
                    continue;
                }

                t4.Add(rect, seg);
            }
        }

        // check if it is possible to shortcut from a to b via while
        // staying inside the slice balls
        // we are collecting all the intersections and tracking the list of balls we're inside
        // at any given point. If list becomes empty, we can't shortcut
        private bool may_shortcut(Point2F a, Point2F b, List<Slice> colliders)
        {
            Line2F path = new Line2F(a, b);
            SortedList<double, List<Slice>> intersections = new SortedList<double, List<Slice>>();
            List<Slice> running_collides = new List<Slice>();

            foreach (Slice s in colliders)
            {
                Line2F insects = s.Ball.LineIntersect(path, GENERAL_TOLERANCE);

                if (insects.p1.IsUndefined && insects.p2.IsUndefined)
                {
                    // no intersections: check if whole path lay inside the circle
                    if (   a.DistanceTo(s.Ball.Center) < s.Ball.Radius + GENERAL_TOLERANCE
                        && b.DistanceTo(s.Ball.Center) < s.Ball.Radius + GENERAL_TOLERANCE)
                        return true;
                }
                else if (insects.p1.IsUndefined || insects.p2.IsUndefined)
                {
                    // single intersection. one of the path ends must be inside the circle, otherwise it is a tangent case
                    // and should be ignored
                    if (a.DistanceTo(s.Ball.Center) < s.Ball.Radius + GENERAL_TOLERANCE)
                    {
                        running_collides.Add(s);
                    }
                    else if (b.DistanceTo(s.Ball.Center) < s.Ball.Radius + GENERAL_TOLERANCE)
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
                        intersections.Add(d, new List<Slice>());
                    intersections[d].Add(s);
                }
                else
                {
                    // double intersection
                    double d = insects.p1.DistanceTo(a);
                    if (! intersections.ContainsKey(d))
                        intersections.Add(d, new List<Slice>());
                    intersections[d].Add(s);

                    d = insects.p2.DistanceTo(a);
                    if (! intersections.ContainsKey(d))
                        intersections.Add(d, new List<Slice>());
                    intersections[d].Add(s);
                }
            }

            if (running_collides.Count == 0) return false;

            foreach (var ins in intersections)
            {
                foreach (Slice s in ins.Value)
                {
                    if (running_collides.Contains(s))
                        running_collides.Remove(s);
                    else
                        running_collides.Add(s);
                }

                if (running_collides.Count == 0 && (ins.Key + GENERAL_TOLERANCE < a.DistanceTo(b)))
                    return false;
            }

            return true;
        }

        private bool may_shortcut(Point2F a, Point2F b, T4 slices)
        {
            T4_rect rect = new T4_rect(Math.Min(a.X, b.X),
                                       Math.Min(a.Y, b.Y),
                                       Math.Max(a.X, b.X),
                                       Math.Max(a.Y, b.Y));

            List<Slice> colliders = new List<Slice>();
            foreach(object obj in slices.Get_colliding_objects(rect))
                colliders.Add((Slice)obj);

            return may_shortcut(a, b, colliders);
        }

        private List<Entity> generate_path(Branch root)
        {
            Slice last_slice = null;

            List<Entity> path = new List<Entity>();

            foreach (Branch b in root.Df_traverse())
            {
                if (b.Entry != null)
                {
                    path.Add(b.Entry);
                }

                for (int sidx = 0; sidx < b.Slices.Count; sidx++)
                {
                    Slice s = b.Slices[sidx];

                    // connect following branch slices
                    if (sidx > 0)
                    {
                        //path.Add(new Line(last_slice.Segments[last_slice.Segments.Count - 1].P2, s.Segments[0].P1));
                    }

                    // emit segments
                    for (int i = 0; i < s.Segments.Count; i++)
                    {
                        // connect segments
                        if (i > 0)
                        {
//                            path.Add(new Line(s.Segments[i - 1].P2, s.Segments[i].P1));
                        }

                        Arc arc = new Arc(s.Segments[i]);
                        arc.Tag = String.Format("me {0:F4}, so {1:F4}", s.Max_engagement, s.Max_engagement / (_cutter_r * 2));
                        path.Add(arc);
                    }
                    last_slice = s;
                }
            }

            return path;
        }

        public List<Entity> run()
        {
            List<Line2F> mat_lines = get_mat_segments();

            Host.log("building tree");
            Branch root = build_tree(mat_lines);
            Host.log("tree built");

            if (root == null)
                return new List<Entity>();

            List<Branch> traverse = root.Df_traverse();

//          foreach (Branch b in traverse)
//              CamBamUI.MainUI.ActiveView.CADFile.ActiveLayer.Entities.Add(b.Curve);

            T4 ready_slices = new T4(_reg_t4.Rect);

            //root.Df_visit(b => roll(b, ready_slices));

            Slice last_slice = null;

            foreach (Branch b in traverse)
            {
                roll(b, ready_slices, ref last_slice);
            }

            return generate_path(root);
        }

        public void Debug_t4(List<Polyline> curves)
        {
            Point3F min = Point3F.Undefined;
            Point3F max = Point3F.Undefined;

            _reg.GetExtrema(ref min, ref max);

            T4 debug_t4 = new T4(new T4_rect(183, -13, 392, 135));

            foreach (Polyline p in curves)
            {
                insert_in_t4(debug_t4, p);
            }

            //new T4_nearest_debugger(CamBamUI.MainUI.ActiveView, t4);
            new T4_collider_debugger(CamBamUI.MainUI.ActiveView, debug_t4);
        }


        public Pocket_generator(Region reg)
        {
            _reg = reg;

            Point3F min = Point3F.Undefined;
            Point3F max = Point3F.Undefined;

            _reg.GetExtrema(ref min, ref max);

            _reg_t4 = new T4(new T4_rect(min.X - 1, min.Y - 1, max.X + 1, max.Y + 1));

            insert_in_t4(_reg_t4, _reg.OuterCurve);
            foreach (Polyline hole in reg.HoleCurves)
            {
                insert_in_t4(_reg_t4, hole);
            }
        }
    }
}
