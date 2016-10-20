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
        private readonly Circle2F _ball;
        private Point2F _p1;
        private Point2F _p2;
        private List<Arc2F> _segments = new List<Arc2F>();
        private readonly double _max_engagement;
        private readonly double _mrr;

        public Circle2F Ball { get { return _ball; } }

        public Point2F Center { get { return _ball.Center; } }
        public double Radius { get { return _ball.Radius; } }
        public double Max_engagement { get { return _max_engagement; } }
        public double Mrr { get { return _mrr; } }
        public List<Arc2F> Segments { get { return _segments; } }

        static public double Calc_max_engagement(Point2F center, double radius, Slice prev_slice)
        {
            double delta_s = Point2F.Distance(center, prev_slice.Center);
            double delta_r = radius - prev_slice.Radius;
            return delta_s + delta_r;
        }

        static public double Calc_mrr(Point2F center, double radius, Slice prev_slice, double cutter_r)
        {
            // http://mathworld.wolfram.com/Lens.html
            // result = area of tested circle - area of inner lens between two circles
            double d = center.DistanceTo(prev_slice.Center);
            double R = prev_slice.Radius + cutter_r;
            double r = radius + cutter_r;

            //XXX: no lens in this case, return 0.
            if (d == 0) return 0;

            double A_lens =    r * r * Math.Acos((d*d + r*r - R*R) / 2.0 / d / r)
                             + R * R * Math.Acos((d*d + R*R - r*r) / 2.0 / d / R)
                             - Math.Sqrt((-d + r + R)*(d + r - R)*(d - r + R)*(d + r +R)) / 2.0;

            return Math.PI * r * r - A_lens;
        }

        public Slice(Point2F center, double radius, Slice prev_slice, RotationDirection dir, double cutter_r)
        {
            _max_engagement = Slice.Calc_max_engagement(center, radius, prev_slice);
            _mrr = Slice.Calc_mrr(center, radius, prev_slice, cutter_r);
            _ball = new Circle2F(center, radius);
            Finalize(prev_slice, dir);
        }

        // temporary lightwidth slice
        public Slice(Point2F center, double radius, Slice prev_slice, double cutter_r)
        {
            _max_engagement = Slice.Calc_max_engagement(center, radius, prev_slice);
            _mrr = Slice.Calc_mrr(center, radius, prev_slice, cutter_r);
            _ball = new Circle2F(center, radius);
        }

        public void Finalize(Slice prev_slice, RotationDirection dir)
        {
            Line2F insects = prev_slice.Ball.CircleIntersect(_ball);

            if (insects.p1.IsUndefined || insects.p2.IsUndefined)
            {
                ;
            }
            else
            {
                Arc2F arc = new Arc2F(_ball.Center, insects.p1, insects.p2, dir);

                if (!arc.VectorInsideArc(new Vector2F(prev_slice.Center, _ball.Center)))
                    arc = new Arc2F(_ball.Center, insects.p2, insects.p1, dir);

                _p1 = arc.P1;
                _p2 = arc.P2;

                _segments.Add(arc);
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

        private double angle_to_start_vector(Point2F p)
        {
            Vector2F v = new Vector2F(_center, p);
            double angle = Math.Atan2(Vector2F.Determinant(_start_vector, v), Vector2F.DotProduct(_start_vector, v));
            return (_dir == RotationDirection.CCW) ? angle : (2.0 * Math.PI - angle);
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
            double d0 = angle_to_start_vector((Point2F)a);
            double d1 = angle_to_start_vector((Point2F)b);

            if (d0 < d1)
                return -1;
            if (d0 > d1)
                return 1;
            return 0;
        }
    }

    public class Pocket_generator
    {
        private const double GENERAL_TOLERANCE = 0.001;
        private const double VORONOI_MARGIN = 1.0;
        private const bool ANALIZE_INNER_INTERSECTIONS = false;

        private readonly Region _reg;
        private readonly T4 _t4;

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
            foreach(object item in _t4.Get_nearest_objects(pt.X, pt.Y))
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

        private List<Slice> find_prev_slices(Branch start)
        {
            return start.Get_upstream_roadblocks();
        }

        private Slice find_prev_slice(Branch start)
        {
            for (Branch b = start.Parent; b != null; b = b.Parent)
            {
                if (b.Slices.Count != 0)
                    return b.Slices[b.Slices.Count - 1];
            }

            return null;
        }

        private List<Circle2F> find_intersecting_balls(List<Circle2F> ballist, Circle2F ball)
        {
            List<Circle2F> result = new List<Circle2F>();
            foreach (Circle2F b in ballist)
            {
                double dist = Point2F.Distance(b.Center, ball.Center);
                if (b.Radius + ball.Radius <= dist + GENERAL_TOLERANCE)
                    continue;
                result.Add(b);
            }
            return result;
        }

        private List<Circle2F> find_intersecting_balls(List<Circle2F> ballist, Arc2F arc)
        {
            List<Circle2F> result = new List<Circle2F>();
            foreach (Circle2F b in ballist)
            {
                double dist = Point2F.Distance(b.Center, arc.Center);
                if (b.Radius + arc.Radius <= dist + GENERAL_TOLERANCE)
                    continue;
                Line2F splitline = arc.CircleIntersect(b);
                if (splitline.p1.IsUndefined && splitline.p2.IsUndefined)
                    continue;
                result.Add(b);
            }
            return result;
        }

        private List<Arc2F> filter_inner_arcs(List<Circle2F> ballist, List<Arc2F> segments)
        {
            List<Arc2F> result = new List<Arc2F>();

            foreach (Arc2F seg in segments)
            {
                bool is_inner = false;
                foreach (Circle2F ball in ballist)
                {
                    // XXX: crude, but will work in most cases
                    if (Point2F.Distance(ball.Center, seg.Midpoint) < ball.Radius)
                    {
                        is_inner = true;
                        break;
                    }
                }
                if (! is_inner)
                    result.Add(seg);
            }
            return result;
        }

        private void roll(Branch branch, List<Slice> ready_slices, RotationDirection dir, double min_segment_length)
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
                Point2F pt = samples[0];
                double radius = mics[0];

                double max_mrr = double.MinValue;
                double max_engagement = double.MinValue;

                double min_mrr = double.MaxValue;
                double min_dist = double.MaxValue;
                double min_engagement = double.MaxValue;

                List<Slice> candidates = find_prev_slices(branch);
                foreach (Slice candidate in candidates)
                {
                    double max_slice_engage = Slice.Calc_max_engagement(pt, radius, candidate);
                    if (max_slice_engage > _max_engagement) continue;

                    double mrr = Slice.Calc_mrr(pt, radius, candidate, _cutter_r);
                    double dist = pt.DistanceTo(candidate.Center);

                    const int algo = 3;

                    switch (algo)
                    {
                    case 0:
                        if (prev_slice == null ||  mrr < min_mrr)
                        {
                            min_mrr = mrr;
                            prev_slice = candidate;
                        }
                        break;

                    case 1:
                        if (prev_slice == null || mrr > max_mrr)
                        {
                            max_mrr = mrr;
                            prev_slice = candidate;
                        }
                        break;

                    case 2:
                        if (prev_slice == null || dist < min_dist)
                        {
                            min_dist = dist;
                            prev_slice = candidate;
                        }
                        break;

                    case 3:
                        if (prev_slice == null || max_slice_engage < min_engagement)
                        {
                            min_engagement = max_slice_engage;
                            prev_slice = candidate;
                        }
                        break;

                    case 4:
                        if (prev_slice == null || max_slice_engage > max_engagement)
                        {
                            max_engagement = max_slice_engage;
                            prev_slice = candidate;
                        }
                        break;
                    }
                }

//              prev_slice = find_prev_slice(branch);

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
                ready_slices.Add(s);
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

                double max_slice_engage = Slice.Calc_max_engagement(pt, radius, prev_slice);

                // queue good candidate and continue
                if (max_slice_engage <= _max_engagement)
                {
                    Slice s = new Slice(pt, radius, prev_slice, _cutter_r);
                    if (pending_slice == null || s.Mrr > pending_slice.Mrr)
                    {
                        pending_slice = s;
                        pending_slice_index = i;
                    }
                    continue;
                }

                // max engagement overshoot, time to dequeue candidate
                if (pending_slice == null)
                {
                    // XXX: will fail here
                    Host.log("nothing is pending !");
                    return;
                }

                pending_slice.Finalize(prev_slice, dir);
                if (pending_slice.Segments.Count == 0)
                {
                    Host.log("Undefined intersection (can't pass thru slot ?). Stopping slicing the branch.");
                    // XXX: should emit last possible slice before unmillable area
                    // now it is not and crudely aborted
                    return;
                }

                branch.Slices.Add(pending_slice);
                ready_slices.Add(pending_slice);
                prev_slice = pending_slice;
                pending_slice = null;
                i = pending_slice_index;    // trace again
            }


            //if ((branch.Is_leaf || branch.Slices.Count == 0) && pending_slice != null)
            //if (pending_slice != null)
            //if (branch.Is_leaf && pending_slice != null && pending_slice.Max_engagement > GENERAL_TOLERANCE)
            if (false)
            {
                pending_slice.Finalize(prev_slice, dir);
                branch.Slices.Add(pending_slice);
                ready_slices.Add(pending_slice);
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
            Host.log("analizing segments");
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
            Host.log("done analizing segments");

            // XXX: startpoint may be undefined

            // craft new artifical start poly
            Branch root = new Branch(null);
            root.Curve.Add(point(start_pt));

            Host.log("got {0} hashes", pool.N_hashes);

            attach_segments(root, pool);

            return root;
        }

        private void populate_t4(Polyline p)
        {
            for (int i = 0; i < p.NumSegments; i++)
            {
                object pi = p.GetSegment(i);    // would be Line2F or Arc2F
                T4_rect rect;
                if (pi is Line2F)
                {
                    Line2F line = ((Line2F)pi);
                    rect = new T4_rect(Math.Min(line.p1.X, line.p2.X),
                                       Math.Min(line.p1.Y, line.p2.Y),
                                       Math.Max(line.p1.X, line.p2.X),
                                       Math.Max(line.p1.Y, line.p2.Y));
                }
                else
                {
                    Point2F min = Point2F.Undefined;
                    Point2F max = Point2F.Undefined;
                    ((Arc2F)pi).GetExtrema(ref min, ref max);
                    rect = new T4_rect(min.X, min.Y, max.X, max.Y);
                }

                _t4.Add(rect, pi);
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
//          {
//              CamBamUI.MainUI.ActiveView.CADFile.ActiveLayer.Entities.Add(b.Curve);
//          }
//
//          return new List<Entity>();

            List<Slice> ready_slices = new List<Slice>();

            foreach (Branch b in traverse)
            {
                roll(b, ready_slices, RotationDirection.CCW, _max_engagement / 2);
            }

            List<Entity> path = new List<Entity>();
            foreach (Branch b in traverse)
            {
                // XXX: for debug
                //path.Add(b.Curve.Clone());
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

        public void Debug_t4()
        {
            new T4_debugger(CamBamUI.MainUI.ActiveView, _t4);
        }

        public Pocket_generator(Region reg)
        {
            _reg = reg;

            Point3F min = Point3F.Undefined;
            Point3F max = Point3F.Undefined;

            _reg.GetExtrema(ref min, ref max);

            _t4 = new T4(new T4_rect(min.X - 1, min.Y - 1, max.X + 1, max.Y + 1));

            populate_t4(_reg.OuterCurve);
            foreach (Polyline hole in reg.HoleCurves)
            {
                populate_t4(hole);
            }
        }
    }
}

//        List<Entity> get_path(Branch branch, List<Circle2F> ballist, bool is_cw, double min_segment_length)
//        {
//            List<Entity> slices = new List<Entity>();
//
//            if (branch.Balls.Count == 0) return slices;
//
//            Circle2F prev_ball;
//
//            int i = 0;
//
//            if (branch.Parent == null)
//            {
//                // XXX: here should be the spiral
//                Circle c = new Circle(branch.Balls[0]);
//                slices.Add(c);
//                prev_ball = branch.Balls[0];
//                ballist.Add(branch.Balls[0]);
//                i = 1;
//            }
//            else
//            {
//                prev_ball = find_nearest_ball(branch.Parent, point(branch.Curve.Points[0].Point));
//            }
//
//            for (; i < branch.Balls.Count; i++)
//            {
//                Circle2F ball = branch.Balls[i];
//                Line2F insects = ball.CircleIntersect(prev_ball);
//
//                if (insects.p1.IsUndefined || insects.p2.IsUndefined)
//                {
//                    // Probably it means there is unreachable slot
//                    // XXX: for  debug
//                    // XXX: throw exception here
//                    slices.Add(new Circle(ball.Center, 100));
//                    slices.Add(new Circle(prev_ball.Center, 120));
//                    Host.log("undefined intersection! i = {0}, r={1}, prev r={2}", i, ball.Radius, prev_ball.Radius);
//                }
//                else
//                {
//                    RotationDirection dir = is_cw ? RotationDirection.CW : RotationDirection.CCW;
//                    Arc2F basic_arc = new Arc2F(ball.Center, insects.p1, insects.p2, dir);
//
//                    //XXX: use dot product here !
//                    if (! basic_arc.VectorInsideArc(new Vector2F(prev_ball.Center, ball.Center)))
//                    {
//                        basic_arc = new Arc2F(ball.Center, insects.p2, insects.p1, dir);        // reverse it
//                    }
//
//                    List<Circle2F> more_insects = find_intersecting_balls(ballist, basic_arc);
//                    // XXX: would this rise exception if prev ball is not there ?
//                    more_insects.Remove(prev_ball);
//
//                    if (more_insects.Count == 0)
//                    {
//                        slices.Add(new Arc(basic_arc));
//                    }
//                    else
//                    {
//                        // just choose the ball with the longest trim chord for now
//
//                        /*
//                        Circle2F trimming_ball = more_insects[0];
//                        double max_chord = 0;
//                        for (int idx=1; idx < more_insects.Count; idx++)
//                        {
//                            Circle2F b = more_insects[idx];
//                            Line2F chord = basic_arc.CircleIntersect(b);
//                            // XXX: wrong !
//                            if (chord.p1.IsUndefined || chord.p2.IsUndefined)
//                                continue;
//
//                            double chord_len = chord.Length();
//                            if (chord_len > max_chord)
//                            {
//                                max_chord = chord_len;
//                                trimming_ball = b;
//                            }
//                        }
//
//                        more_insects.Clear();
//                        more_insects.Add(trimming_ball);
//                        */
//
//
//                        /*
//                        List<Arc2F> segments = segment_arc_by_balls(basic_arc, more_insects, slices);
//
//                        foreach (Arc2F seg in segments)
//                        {
//                            if (seg.GetPerimeter() > basic_arc.GetPerimeter())
//                            {
//                                Host.log("perimeter is bigger !");
//                            }
//
//                            if (seg.GetPerimeter() > min_segment_length)
//                            {
////                                slices.Add(new Arc(seg));
//                            }
//                        }
//                        */
//
//                        slices.Add(new Arc(basic_arc));
//                    }
//                }
//
//                prev_ball = ball;
//                ballist.Add(ball);
//            }
//            return slices;
//        }
