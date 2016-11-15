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
    class Pocket_generator
    {
        private const double GENERAL_TOLERANCE = 0.001;
        private const double VORONOI_MARGIN = 1.0;
        private const bool ANALIZE_INNER_INTERSECTIONS = false;
        private const double ENGAGEMENT_TOLERANCE_PERCENTAGE = 0.001;  // 0.1 %

        private readonly Region _reg;
        private readonly T4 _reg_t4;

        private double _cutter_r = 1.5;
        private double _margin = 0;
        private double _max_engagement = 3.0 * 0.4;
        private double _min_engagement = 3.0 * 0.1;
        private Point2F _startpoint = Point2F.Undefined;
        private bool _return_to_base = false;

        private RotationDirection _dir = RotationDirection.CW;

        public double Cutter_d        {set { _cutter_r = value / 2.0;}}
        public double Max_engagement  {set { _max_engagement = value; } }
        public double Min_engagement  {set { _min_engagement = value; } }
        public double Margin          {set { _margin = value; } }
        public Point2F Startpoint     {set { _startpoint = value; } }
        public bool Return_to_base    {set { _return_to_base = value; } }
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

        private bool is_line_inside_region(Line2F line, bool should_analize_inner_intersections)
        {
            if (!_reg.OuterCurve.PointInPolyline(line.p1, GENERAL_TOLERANCE)) return false;     // p1 is outside of outer curve boundary
            if (!_reg.OuterCurve.PointInPolyline(line.p2, GENERAL_TOLERANCE)) return false;  // p2 is outside of outer curve boundary
            if (should_analize_inner_intersections && _reg.OuterCurve.LineIntersections(line, GENERAL_TOLERANCE).Length != 0) return false; // both endpoints are inside, but there are intersections, outer curve must be concave

            foreach (Polyline hole in _reg.HoleCurves)
            {
                if (hole.PointInPolyline(line.p1, GENERAL_TOLERANCE)) return false;  // p1 is inside hole
                if (hole.PointInPolyline(line.p2, GENERAL_TOLERANCE)) return false;  // p2 is inside hole
                if (should_analize_inner_intersections && hole.LineIntersections(line, GENERAL_TOLERANCE).Length != 0) return false; // p1, p2 are outside hole, but there are intersections
            }
            return true;
        }

        private List<Line2F> get_mat_segments()
        {
            List<Point2F> plist = new List<Point2F>();

            plist.AddRange(sample_curve(this._reg.OuterCurve, _cutter_r / 10));
            foreach (Polyline p in this._reg.HoleCurves)
                plist.AddRange(sample_curve(p, _cutter_r / 10));

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

            ys[hackpoint_idx] -= GENERAL_TOLERANCE;

            min_x -= VORONOI_MARGIN;
            max_x += VORONOI_MARGIN;
            min_y -= VORONOI_MARGIN;
            max_y += VORONOI_MARGIN;

            List<GraphEdge> edges = new Voronoi(GENERAL_TOLERANCE).generateVoronoi(xs, ys, min_x, max_x, min_y, max_y);

            Host.log("voronoi partitioning completed. got {0} edges", edges.Count);

            List<Line2F> inner_segments = new List<Line2F>();

            foreach (GraphEdge e in edges)
            {
                Line2F seg = new Line2F(e.x1, e.y1, e.x2, e.y2);

                if (seg.Length() < GENERAL_TOLERANCE) continue;    // extra small segment, discard
                if (! is_line_inside_region(seg, ANALIZE_INNER_INTERSECTIONS)) continue;
                inner_segments.Add(seg);
            }

            Host.log("got {0} inner segments", inner_segments.Count);

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

            // account for margin in just one subrtract. Nice !
            return radius - _cutter_r - _margin;
        }

        private Slice find_prev_slice(Branch branch, Point2F pt, double radius, T4 ready_slices)
        {
            Slice prev_slice = null;

            double min_engage = double.MaxValue;

            List<Slice> candidates = branch.Get_upstream_roadblocks();
            foreach (Slice candidate in candidates)
            {
                Slice s = new Slice(candidate, pt, radius, _dir);
                if (! s.Is_undefined)
                    s.Refine(find_colliding_slices(s, ready_slices), _cutter_r);

                double slice_engage = s.Max_engagement;
                if (slice_engage > _max_engagement)
                    continue;

                if (prev_slice == null || slice_engage < min_engage)
                {
                    min_engage = slice_engage;
                    prev_slice = candidate;
                }
            }

            return prev_slice;
        }

        private List<Slice> find_colliding_slices(Slice s, T4 ready_slices)
        {
            Point2F min = Point2F.Undefined;
            Point2F max = Point2F.Undefined;
            s.Get_extrema(ref min, ref max);
            T4_rect rect = new T4_rect(min.X, min.Y, max.X, max.Y);
            List<Slice> result = new List<Slice>();
            // TODO: is there a way to do it without repacking ?
            foreach (object obj in ready_slices.Get_colliding_objects(rect))
                result.Add((Slice)obj);
            return result;
        }

        // find least common ancestor for the both branches
        Entity switch_branch(Slice dst, Slice src, T4 ready_slices, Point2F dst_pt, Point2F src_pt)
        {
            List<Slice> path = new List<Slice>();

            Point2F current = src_pt.IsUndefined ? src.Segments[src.Segments.Count - 1].P2 : src_pt;
            Point2F end = dst_pt.IsUndefined ? dst.Segments[0].P1 : dst_pt;

            // continuation
            if (dst.Prev == src)
            {
                return (Entity)new Line(current, end);
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

        Entity switch_branch(Slice dst, Slice src, T4 ready_slices)
        {
            return switch_branch(dst, src, ready_slices, Point2F.Undefined, Point2F.Undefined);
        }

        private void roll(Branch branch, T4 ready_slices, ref Slice last_slice)
        {
            Slice prev_slice = null;

            if (branch.Curve.Points.Count == 0)
                throw new Exception("branch with the empty curve");

            Point2F start_pt = branch.Curve.Start;
            double start_radius = get_mic_radius(start_pt);

            if (branch.Parent != null)
            {
                // non-initial slice
                prev_slice = find_prev_slice(branch, start_pt, start_radius, ready_slices);
                if (prev_slice == null)
                {
                    Host.warn("failed to attach branch");
                    return;
                }
            }
            else
            {
                Slice s = new Slice(start_pt, start_radius, _dir);
                branch.Slices.Add(s);
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
                    if (! s.Is_undefined)
                    {
                        s.Refine(find_colliding_slices(s, ready_slices), _cutter_r);
                    }

                    // prefer slight undershoot within the engagement tolerance
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

                    Point2F other = branch.Curve.Get_parametric_pt(left == mid ? right : left);

                    if (pt.DistanceTo(other) < GENERAL_TOLERANCE)
                    {
                        left = mid;
                        break;
                    }
                }

                if (s.Radius < _cutter_r) return;

                double err = (s.Max_engagement - _max_engagement) / _max_engagement;

                if (err > ENGAGEMENT_TOLERANCE_PERCENTAGE * 10)
                {
                    Host.err("failed to create slice within stepover limit. stopping slicing the branch");
                    return;
                }

                if (s.Max_engagement < _min_engagement) return;

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
            Segpool pool = new Segpool(segments.Count, GENERAL_TOLERANCE);
            Branch root = new Branch(null);
            Point2F tree_start = Point2F.Undefined;

            Host.log("analyzing segments");
            if (_startpoint.IsUndefined)
            {
                // automatic startpoint, choose the start segment - the one with the largest mic
                double max_r = double.MinValue;

                foreach (Line2F line in segments)
                {
                    double r1 = get_mic_radius(line.p1);
                    double r2 = get_mic_radius(line.p2);
                    // discard segments unmillable from both sides
                    if (r1 < _cutter_r && r2 < _cutter_r)
                        continue;

                    pool.Add(line);

                    if (r1 > max_r)
                    {
                        max_r = r1;
                        tree_start = line.p1;
                    }
                    if (r2 > max_r)
                    {
                        max_r = r2;
                        tree_start = line.p2;
                    }
                }
            }
            else
            {
                // manual startpoint, seek the segment with the closest end to startpoint
                if (! is_line_inside_region(new Line2F(_startpoint, _startpoint), false))
                {
                    Host.warn("startpoint is outside the pocket");
                    return null;
                }
                if (get_mic_radius(_startpoint) < _cutter_r)
                {
                    Host.warn("startpoint radius < cutter radius");
                    return null;
                }

                // insert startpoing to root poly, it would be connected to seg_start later
                root.Curve.Add(_startpoint);

                double min_dist = double.MaxValue;

                foreach (Line2F line in segments)
                {
                    double r1 = get_mic_radius(line.p1);
                    double r2 = get_mic_radius(line.p2);
                    // discard segments unmillable from both sides
                    if (r1 < _cutter_r && r2 < _cutter_r)
                        continue;

                    pool.Add(line);

                    double dist1 = _startpoint.DistanceTo(line.p1);
                    double dist2 = _startpoint.DistanceTo(line.p2);

                    if (dist1 < min_dist && is_line_inside_region(new Line2F(_startpoint, line.p1), true))
                    {
                        min_dist = dist1;
                        tree_start = line.p1;
                    }
                    if (dist2 < min_dist && is_line_inside_region(new Line2F(_startpoint, line.p2), true))
                    {
                        min_dist = dist2;
                        tree_start = line.p2;
                    }
                }
            }

            if (tree_start.IsUndefined)
            {
                Host.warn("failed to choose tree start point");
                return null;
            }

            Host.log("done analyzing segments");
            Host.log("got {0} hashes", pool.N_hashes);

            root.Curve.Add(tree_start);
            attach_segments(root, pool);
            return root;
        }

        private void insert_in_t4(T4 t4, Slice slice)
        {
            Point2F min = Point2F.Undefined;
            Point2F max = Point2F.Undefined;
            slice.Get_ball_extrema(ref min, ref max);
            T4_rect rect = new T4_rect(min.X, min.Y, max.X, max.Y);
            t4.Add(rect, slice);
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
                    throw new Exception("unknown segment type");
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

            if (running_collides.Count == 0)
                return false;

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

        private List<Entity> generate_path(List<Branch> traverse, T4 ready_slices)
        {
            Slice last_slice = null;

            List<Entity> path = new List<Entity>();

            Slice root_slice = traverse[0].Slices[0];

            // emit spiral toolpath for root
            Polyline spiral = SpiralGenerator.GenerateFlatSpiral(root_slice.Center, root_slice.Segments[0].P1, _max_engagement, _dir);
            path.Add(spiral);

            for (int bidx = 0; bidx < traverse.Count; bidx++)
            {
                Branch b = traverse[bidx];

                if (b.Entry != null)
                {
                    path.Add((Entity)b.Entry);
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
                    for (int segidx = 0; segidx < s.Segments.Count; segidx++)
                    {
                        // connect segments
                        if (segidx > 0)
                        {
                            path.Add(new Line(s.Segments[segidx - 1].P2, s.Segments[segidx].P1));
                        }

                        Arc arc = new Arc(s.Segments[segidx]);
                        arc.Tag = String.Format("me {0:F4}, so {1:F4}", s.Max_engagement, s.Max_engagement / (_cutter_r * 2));
                        path.Add(arc);
                    }
                    last_slice = s;
                }
            }

            // hacky
            if (_return_to_base)
            {
                path.Add(switch_branch(root_slice, last_slice, ready_slices, root_slice.Segments[0].Center, Point2F.Undefined));
            }

            return path;
        }

        public List<Entity> run()
        {
            List<Line2F> mat_lines = get_mat_segments();

            Host.log("building tree");
            Branch root = build_tree(mat_lines);
            if (root == null)
            {
                Host.warn("failed to build tree");
                return new List<Entity>();
            }

            List<Branch> traverse = root.Df_traverse();

//          foreach (Branch b in traverse)
//              CamBamUI.MainUI.ActiveView.CADFile.ActiveLayer.Entities.Add(b.Curve);

            T4 ready_slices = new T4(_reg_t4.Rect);
            Slice last_slice = null;

            Host.log("generating slices");
            foreach (Branch b in traverse)
                roll(b, ready_slices, ref last_slice);

            Host.log("generating path");
            return generate_path(traverse, ready_slices);
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
