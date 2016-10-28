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
    class Slice
    {
        // TODO: ball or arc: choose only one representation
        private Circle2F _ball;
        private Arc2F _arc;
        private double _max_engagement;
        private List<Arc2F> _segments = new List<Arc2F>();
        private readonly double _mrr;
        private Slice _prev_slice;

        public Circle2F Ball { get { return _ball; } }
        public Arc2F Arc { get { return _arc; } }

        public Point2F Center { get { return _ball.Center; } }
        public double Radius { get { return _ball.Radius; } }
        public double Max_engagement { get { return _max_engagement; } }
        public double Mrr { get { return _mrr; } }
        //public Slice Prev { get { return _prev_slice; } }
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

        static public double Calc_mrr(Point2F center, double radius, Slice prev_slice, double cutter_r)
        {
            double R = prev_slice.Radius + cutter_r;
            double r = radius;

            double A_lens = Calc_lens(prev_slice.Center, prev_slice.Radius, center, radius);
            if (A_lens == 0) return 0;

            return Math.PI * r * r - A_lens;
        }

        /*
        public Slice(Point2F center, double radius, Slice prev_slice, RotationDirection dir, double cutter_r)
        {
            _max_engagement = Slice.Calc_max_engagement(center, radius, prev_slice);
            _mrr = Slice.Calc_mrr(center, radius, prev_slice, cutter_r);
            _ball = new Circle2F(center, radius);
            Finalize(prev_slice, dir);
        }
        */

        // temporary lightwidth slice
        public Slice(Point2F center, double radius, Slice prev_slice, double cutter_r)
        {
            _max_engagement = Slice.Calc_max_engagement(center, radius, prev_slice);
            _mrr = Slice.Calc_mrr(center, radius, prev_slice, cutter_r);
            _ball = new Circle2F(center, radius);
        }

        public bool Finalize(Slice prev_slice, RotationDirection dir)
        {
            Line2F insects = prev_slice.Ball.CircleIntersect(_ball);

            if (insects.p1.IsUndefined || insects.p2.IsUndefined)
                return false;

            _prev_slice = prev_slice;

            _arc = new Arc2F(_ball.Center, insects.p1, insects.p2, dir);

            if (! _arc.VectorInsideArc(new Vector2F(prev_slice.Center, _ball.Center)))
                _arc = new Arc2F(_ball.Center, insects.p2, insects.p1, dir);

            return true;
        }

        public void Refine(Slice prev_slice, List<Slice> colliding_slices, double end_clearance)
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
                if (s == prev_slice) continue;  // no reason to process it

                // TODO: choose the best option: arc-arc intersection or arc-circle
                Line2F secant = _arc.ArcIntersect(s.Arc);
                //Line2F secant = _arc.CircleIntersect(s.Ball);

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
                    if (arc.Midpoint.DistanceTo(s.Ball.Center) >= s.Ball.Radius) continue;

                    max_sweep = sweep;
                    max_secant = secant;
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
                // TODO: is it really needed ? should we apply opposite derating is base engagement is valid ?
                if (removed.VectorInsideArc(new Vector2F(_arc.Center, _arc.Midpoint)))
                {
                    double e0 = prev_slice.Center.DistanceTo(max_secant.p1) - prev_slice.Radius;
                    double e1 = prev_slice.Center.DistanceTo(max_secant.p2) - prev_slice.Radius;

                    _max_engagement += Math.Max(e0, e1);
                    _max_engagement /= 2.0;
                }
            }
        }

        public Slice(Point2F center, double radius, RotationDirection dir)
        {
            _ball = new Circle2F(center, radius);
            _max_engagement = 0;
            // XXX: hack, just for now
            Arc2F arc0 = new Arc2F(center, radius, 0, 120);
            Arc2F arc1 = new Arc2F(center, radius, 120, 120);
            Arc2F arc2 = new Arc2F(center, radius, 240, 120);
            _segments.Add(arc0);
            _segments.Add(arc1);
            _segments.Add(arc2);
        }
    }

    class Branch
    {
        public readonly Polyline Curve = new Polyline();
        public readonly Branch Parent = null;
        public readonly List<Branch> Children = new List<Branch>();
        public readonly List<Slice> Slices = new List<Slice>();
        public string Debug = "";

        public bool Is_leaf { get { return Children.Count == 0; } }

        private double _deep_distance = 0;

        public List<Branch> Df_traverse()  //
        {
            List<Branch> result = new List<Branch>();
            result.Add(this);
            foreach (Branch b in Children)
                result.AddRange(b.Df_traverse());
            return result;
        }

        // NOTE: deep distance is memoized, so this should be called only on finalized branch
        public double Deep_distance()
        {
            if (_deep_distance != 0)
                return _deep_distance;

            _deep_distance = Curve.GetPerimeter();
            foreach (Branch b in Children)
                _deep_distance += b.Deep_distance();

            return _deep_distance;
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

    // special structure for a fast joining of chained line segments
    // segments (references to them) are stored in dictionary under the long integer key, crafted from
    // their coordinates. segment is stored twice, under the keys for start and end points.
    // lookup should give a small range of nearby segments, then finepicked by the distance compare
    // this way we may find next segment(s) in chain in almost O(1)

    // pull operation removes chained segments from pool and return them.

    // segments may map to different keys if their coordinates are rounded to neighbour integers,
    // this is remedied by storing 4 hashes instead of one, with x and y coordinates floored and
    // ceiled. it corresponds to 4 nearby cells in 2d grid
    class Segpool
    {
        private Dictionary<ulong, List<Line2F>> _pool;
        private readonly double _tolerance;

        public int N_hashes { get { return _pool.Count; } }

        private ulong[] hash(Point2F pt)
        {
            double hashscale = 1 / (_tolerance * 10);
            double x = pt.X * hashscale;
            double y = pt.Y * hashscale;

            return new ulong[]
            {
                    ((ulong)Math.Floor(y) << 32) | (ulong)Math.Floor(x),
                    ((ulong)Math.Floor(y) << 32) | (ulong)Math.Ceiling(x),
                    ((ulong)Math.Ceiling(y) << 32) | (ulong)Math.Floor(x),
                    ((ulong)Math.Ceiling(y) << 32) | (ulong)Math.Ceiling(x),
            };
        }

        private void insert_seg(ulong h, Line2F seg)
        {
            if (!_pool.ContainsKey(h))
                _pool[h] = new List<Line2F>();
            if (!_pool[h].Contains(seg))
                _pool[h].Add(seg);
        }

        private void remove_seg(ulong h, Line2F seg)
        {
            if (_pool.ContainsKey(h))
                _pool[h].Remove(seg);
        }

        public void Add(Line2F seg)
        {
            ulong[] h1 = hash(seg.p1);

            insert_seg(h1[0], seg);
            insert_seg(h1[1], seg);
            insert_seg(h1[2], seg);
            insert_seg(h1[3], seg);

            ulong[] h2 = hash(seg.p2);

            insert_seg(h2[0], seg);
            insert_seg(h2[1], seg);
            insert_seg(h2[2], seg);
            insert_seg(h2[3], seg);
        }

        public void Remove(Line2F seg)
        {
            ulong[] h1 = hash(seg.p1);

            remove_seg(h1[0], seg);
            remove_seg(h1[1], seg);
            remove_seg(h1[2], seg);
            remove_seg(h1[3], seg);

            ulong[] h2 = hash(seg.p2);

            remove_seg(h2[0], seg);
            remove_seg(h2[1], seg);
            remove_seg(h2[2], seg);
            remove_seg(h2[3], seg);
        }

        public List<Point2F> Pull_follow_points(Point2F join_pt)
        {
            List<Point2F> followers = new List<Point2F>();
            List<Line2F> processed = new List<Line2F>();

            ulong[] h = hash(join_pt);

            for (int i = 0; i < 4; i++)
            {
                if (!_pool.ContainsKey(h[i]))
                    continue;

                foreach (Line2F seg in _pool[h[i]])
                {
                    if (processed.Contains(seg))
                        continue;  // already got it

                    if (join_pt.DistanceTo(seg.p1) < _tolerance)
                    {
                        followers.Add(seg.p2);
                        processed.Add(seg);
                    }
                    else if (join_pt.DistanceTo(seg.p2) < _tolerance)
                    {
                        followers.Add(seg.p1);
                        processed.Add(seg);
                    }
                }
            }

            foreach (Line2F seg in processed)
            {
                Remove(seg);
            }

            return followers;
        }

        public Segpool(int capacity, double tolerance)
        {
            _pool = new Dictionary<ulong, List<Line2F>>(capacity * 2 * 4);
            _tolerance = tolerance;
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

    enum prev_slice_algo
    {
        MIN_MRR,
        MAX_MRR,
        MIN_DIST,
        MAX_DIST,
        MIN_ENGAGE,
        MAX_ENGAGE
    };

    enum roll_algo
    {
        MAXIMIZE_ENGAGEMENT,
        MAXIMIZE_MRR,
    };

    class Pocket_generator
    {
        private const double GENERAL_TOLERANCE = 0.001;
        private const double VORONOI_MARGIN = 1.0;
        private const bool ANALIZE_INNER_INTERSECTIONS = false;
        private const double MIN_LEAF_ENGAGEMENT_RATIO = 1; // XXX: effectively disabled now

        private const prev_slice_algo PREV_SLICE_ALGO = prev_slice_algo.MIN_ENGAGE;
        private const roll_algo ROLL_ALGO = roll_algo.MAXIMIZE_ENGAGEMENT;

        private readonly Region _reg;
        private readonly T4 _reg_t4;

        private double _cutter_r = 1.5;
        private double _max_engagement = 3.0 * 0.4;
        private double _sample_distance = 3.0 * 0.4 * 0.1;

        public double cutter_d        {set { _cutter_r = value / 2.0;}}
        public double max_engagement  {set { _max_engagement = value; } }
        public double sample_distance {set { _sample_distance = value; } }

        private Point3F point(Point2F p2)
        {
            return new Point3F(p2.X, p2.Y, 0);
        }

        private Point2F point(Point3F p3)
        {
            return new Point2F(p3.X, p3.Y);
        }

        private enum st
        {
            SEEKING_PASSABLE_START,
            SEEKING_UNPASSABLE_MIDDLE,
            SEEKING_PASSABLE_END,
            FLUSHING_END,
        };

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

        private Slice find_prev_slice(Branch branch, Point2F pt, double radius, T4 ready_slices, prev_slice_algo algo)
        {
            Slice prev_slice = null;

            double max_mrr = double.MinValue;
            double max_dist = double.MinValue;
            double max_engage = double.MinValue;

            double min_mrr = double.MaxValue;
            double min_dist = double.MaxValue;
            double min_engage = double.MaxValue;

            List<Slice> candidates = branch.Get_upstream_roadblocks();
            foreach (Slice candidate in candidates)
            {
                Slice s = new Slice(pt, radius, candidate, _cutter_r);
                // XXX: dir is for now
                s.Finalize(candidate, RotationDirection.CCW);
                s.Refine(candidate, find_colliding_slices(s, ready_slices), _cutter_r);

                //double slice_engage = Slice.Calc_max_engagement(pt, radius, candidate);
                double slice_engage = s.Max_engagement;
                if (slice_engage > _max_engagement)
                    continue;

                double mrr = s.Mrr;
                //double mrr = Slice.Calc_mrr(pt, radius, candidate, _cutter_r);
                double dist = pt.DistanceTo(candidate.Center);

                switch (algo)
                {
                    case prev_slice_algo.MIN_MRR:
                        if (prev_slice == null || mrr < min_mrr)
                        {
                            min_mrr = mrr;
                            prev_slice = candidate;
                        }
                        break;

                    case prev_slice_algo.MAX_MRR:
                        if (prev_slice == null || mrr > max_mrr)
                        {
                            max_mrr = mrr;
                            prev_slice = candidate;
                        }
                        break;

                    case prev_slice_algo.MIN_DIST:
                        if (prev_slice == null || dist < min_dist)
                        {
                            min_dist = dist;
                            prev_slice = candidate;
                        }
                        break;

                    case prev_slice_algo.MAX_DIST:
                        if (prev_slice == null || dist > max_dist)
                        {
                            max_dist = dist;
                            prev_slice = candidate;
                        }
                        break;

                    case prev_slice_algo.MIN_ENGAGE:
                        if (prev_slice == null || slice_engage < min_engage)
                        {
                            min_engage = slice_engage;
                            prev_slice = candidate;
                        }
                        break;

                    case prev_slice_algo.MAX_ENGAGE:
                        if (prev_slice == null || slice_engage > max_engage)
                        {
                            max_engage = slice_engage;
                            prev_slice = candidate;
                        }
                        break;
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

        private void roll(Branch branch, T4 ready_slices, RotationDirection dir)
        {
            List <Point2F> samples = sample_curve(branch.Curve, _sample_distance);

            // calculate mics once to prevent expensive recalculations
            double[] mics = new double[samples.Count];

            for (int mic_idx = 0; mic_idx < samples.Count; mic_idx++)
            {
                mics[mic_idx] = get_mic_radius(samples[mic_idx]);
            }

            Slice prev_slice = null;
            Slice pending_slice = null;
            int pending_slice_index = 0;

            int i = 0;

            // initial slice
            if (branch.Parent != null)
            {
                prev_slice = find_prev_slice(branch, samples[0], mics[0], ready_slices, PREV_SLICE_ALGO);

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
                Point2F pt = samples[0];
                double radius = mics[0];

                Slice s = new Slice(pt, radius, dir);
                branch.Slices.Add(s);
                // XXX: for now
                // insert_in_t4(ready_slices, s.Ball);
                prev_slice = s;
                i += 1;
            }

            // XXX: lerp instead of skipping should be nice
            for (; i < samples.Count; i++)
            {
                Point2F pt = samples[i];
                double radius = mics[i];

                if (radius < _cutter_r)
                    continue;

                //double slice_engage = Slice.Calc_max_engagement(pt, radius, prev_slice);

                Slice s = new Slice(pt, radius, prev_slice, _cutter_r);
                s.Finalize(prev_slice, dir);
                s.Refine(prev_slice, find_colliding_slices(s, ready_slices), _cutter_r);

                // queue good candidate and continue
                if (s.Max_engagement <= _max_engagement)
                {
                    //Slice s = new Slice(pt, radius, prev_slice, _cutter_r);
                    bool choose_it  = false;

                    if (pending_slice == null)
                    {
                        choose_it = true;
                    }
                    else
                    {
                        if (ROLL_ALGO == roll_algo.MAXIMIZE_ENGAGEMENT)
                        {
                            if (s.Max_engagement >= pending_slice.Max_engagement)
                                choose_it = true;
                        }
                        else if (ROLL_ALGO == roll_algo.MAXIMIZE_MRR)
                        {
                            if (s.Mrr >= pending_slice.Mrr)
                                choose_it = true;
                        }
                    }

                    if (choose_it)
                    {
                        pending_slice = s;
                        pending_slice_index = i;
                    }
                    continue;
                }

                // max engagement overshoot, time to dequeue candidate
                if (pending_slice == null)
                {
                    Host.log("No pending slice before stepover overshoot. Stopping slicing the branch.");
                    return;
                }

//              if (! pending_slice.Finalize(prev_slice, dir))
//              {
//                  Host.log("Can't connect slice to previous (can't pass thru slot ?). Stopping slicing the branch.");
//                  // XXX: should emit last possible slice before unmillable area
//                  // now it is not and crudely aborted
//                  return;
//              }

                //pending_slice.Refine(prev_slice, find_colliding_slices(pending_slice, ready_slices));

                branch.Slices.Add(pending_slice);
                insert_in_t4(ready_slices, pending_slice);
                prev_slice = pending_slice;
                pending_slice = null;
                i = pending_slice_index;    // trace again
            }

            if (branch.Is_leaf && pending_slice != null && pending_slice.Max_engagement > prev_slice.Max_engagement * MIN_LEAF_ENGAGEMENT_RATIO)
            {
//              if (! pending_slice.Finalize(prev_slice, dir))
//              {
//                  Host.log("Failed to finalize last slice");
//                  return;
//              }

//              pending_slice.Refine(prev_slice, find_colliding_slices(pending_slice, ready_slices));
                branch.Slices.Add(pending_slice);
                insert_in_t4(ready_slices, pending_slice);
            }
        }

        //XXX: slices are for debug
        private List<Arc2F> segment_arc_by_balls(Arc2F basic, List<Circle2F> ballist, List<Entity> slices)
        {
            // split arc more to reduce air time

            List<Point2F> insect_points = new List<Point2F>();
            foreach (Circle2F b in ballist)
            {
                Line2F insect_line = basic.CircleIntersect(b);
                if (!insect_line.p1.IsUndefined)
                    insect_points.Add(insect_line.p1);
                if (!insect_line.p2.IsUndefined)
                    insect_points.Add(insect_line.p2);
            }


            Point2F[] insect_array = insect_points.ToArray();
            Array.Sort(insect_array, new Sweep_comparer(basic.P1, basic.Center, basic.Direction));


            /*
            foreach (Point2F pt in insect_array)
            {
                slices.Add(new Circle(pt, 1));
            }
            */

            List<Arc2F> segments = new List<Arc2F>();

            // XXX: wrong, wrong, wrong
            Arc2F remain = basic;
            for (int idx = 0; idx < insect_array.Length; idx++)
            {
                Arc2F[] split = remain.SplitAtPoint(insect_array[idx]);
//                if (Point2F.Distance(split[0].P1, split[0].P2) > GENERAL_TOLERANCE)
//                {
                    segments.Add(split[0]);
                    remain = split[1];
//                }
            }

//            if (Point2F.Distance(remain.P1, remain.P2) > GENERAL_TOLERANCE)
                segments.Add(remain);

            // XXX: Good segments may be filtered out !
             //segments = filter_inner_arcs(ballist, segments);

            //slices.Add(new Arc(basic_arc));

            return segments;
        }

        private void attach_segments(Branch me, Segpool pool)
        {
            Point2F running_end = (Point2F)me.Curve.Points[me.Curve.Points.Count - 1].Point;
            List<Point2F> followers;

            while (true)
            {
                followers = pool.Pull_follow_points(running_end);

                if (followers.Count != 1)
                    break;

                running_end = followers[0];
                me.Curve.Add(point(running_end));   // continuation
            }

            if (followers.Count == 0) return; // end of branch, go out

            foreach (Point2F pt in followers)
            {
                Branch b = new Branch(me);
                b.Curve.Add(point(running_end));
                b.Curve.Add(point(pt));
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
            root.Curve.Add(point(start_pt));
            attach_segments(root, pool);

            return root;
        }

        private void insert_in_t4(T4 t4, object primitive)
        {
            T4_rect rect;
            if (primitive is Line2F)
            {
                Line2F line = ((Line2F)primitive);
                rect = new T4_rect(Math.Min(line.p1.X, line.p2.X),
                                    Math.Min(line.p1.Y, line.p2.Y),
                                    Math.Max(line.p1.X, line.p2.X),
                                    Math.Max(line.p1.Y, line.p2.Y));
            }
            else if (primitive is Arc2F)
            {
                Point2F min = Point2F.Undefined;
                Point2F max = Point2F.Undefined;
                ((Arc2F)primitive).GetExtrema(ref min, ref max);
                rect = new T4_rect(min.X, min.Y, max.X, max.Y);
            }
            else if (primitive is Circle2F)
            {
                Circle2F circle = ((Circle2F)primitive);
                Point2F center = circle.Center;
                double radius = circle.Radius;
                rect = new T4_rect(center.X - radius, center.Y - radius, center.X + radius, center.Y + radius);
            }
            else
            {
                return;
            }

            t4.Add(rect, primitive);
        }

        private void insert_in_t4(T4 t4, Slice slice)
        {
            Point2F min = Point2F.Undefined;
            Point2F max = Point2F.Undefined;
            slice.Arc.GetExtrema(ref min, ref max);
            t4.Add(new T4_rect(min.X, min.Y, max.X, max.Y), slice);
        }

        private void insert_in_t4(T4 t4, Polyline p)
        {
            for (int i = 0; i < p.NumSegments; i++)
            {
                insert_in_t4(t4, p.GetSegment(i));
            }
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

            foreach (Branch b in traverse)
            {
                roll(b, ready_slices, RotationDirection.CCW);
            }

            List<Entity> path = new List<Entity>();
            foreach (Branch b in traverse)
            {
                // XXX: for debug
                Entity p = b.Curve.Clone();
                p.Tag = b.Get_parents().Count.ToString();
                p.Tag += b.Debug;
                //CamBamUI.MainUI.ActiveView.CADFile.ActiveLayer.Entities.Add(p);
                path.Add(p);
                foreach (Slice s in b.Slices)
                {
                    foreach (Arc2F seg in s.Segments)
                    {
                        Arc arc = new Arc(seg);
                        arc.Tag = String.Format("me {0:F4}, so {1:F4}, mrr {2:F4}", s.Max_engagement, s.Max_engagement / (_cutter_r * 2), s.Mrr);
                        path.Add(arc);
                    }
                }
            }

            return path;
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
