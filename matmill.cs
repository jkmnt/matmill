using System;
using System.Collections.Generic;

using CamBam.CAD;
using CamBam.Geom;

using Tree4;
using Voronoi2;
using Geom;

namespace Matmill
{
    enum Pocket_path_item_type
    {
        SLICE,
        SPIRAL,
        CHORD,
        SMOOTH_CHORD,
        BRANCH_ENTRY,
        SLICE_SHORTCUT,
        RETURN_TO_BASE,
        DEBUG_MEDIAL_AXIS,
    }

    class Pocket_path : List<Pocket_path_item> { }

    class Pocket_path_item: Polyline
    {
        public readonly Pocket_path_item_type Item_type;

        public Pocket_path_item(Pocket_path_item_type type) : base()
        {
            Item_type = type;
        }

        public Pocket_path_item(Pocket_path_item_type type, int i) : base(i)
        {
            Item_type = type;
        }

        public Pocket_path_item(Pocket_path_item_type type, Polyline p) : base(p)
        {
            Item_type = type;
        }

        public void Add(Point2F pt)
        {
            base.Add(new Point3F(pt.X, pt.Y, 0));
        }

        public void Add(Curve curve)
        {
            foreach (Point2F pt in curve.Points)
                this.Add(pt);
        }

        public void Add(Biarc2d biarc, double tolerance)
        {
            if (biarc.Seg1 is Arc2F)
                this.Add((Arc2F)biarc.Seg1, tolerance);
            else
                this.Add((Line2F)biarc.Seg1, tolerance);

            if (biarc.Seg2 is Arc2F)
                this.Add((Arc2F)biarc.Seg2, tolerance);
            else
                this.Add((Line2F)biarc.Seg2, tolerance);
        }
    }

    class Pocket_generator
    {
        private const double VORONOI_MARGIN = 1.0;
        private const bool ANALIZE_INNER_INTERSECTIONS = false;
        private const double TED_TOLERANCE_PERCENTAGE = 0.001;  // 0.1 %
        private const double FINAL_ALLOWED_TED_OVERSHOOT_PERCENTAGE = 0.03;  // 3 %

        private bool DROP_BRANCH_ON_OVERSHOOT = true;

        private readonly Polyline _outline;
        private readonly Polyline[] _islands;

        private readonly T4 _reg_t4;

        private double _general_tolerance = 0.001;
        private double _tool_r = 1.5;
        private double _margin = 0;
        private double _max_ted = 3.0 * 0.4;
        private double _min_ted = 3.0 * 0.1;
        private Point2F _startpoint = Point2F.Undefined;
        private RotationDirection _dir = RotationDirection.CW;
        private bool _should_smooth_chords = false;
        private bool _should_emit_debug_medial_axis = false;
        private double _slice_leadin_angle = 3 * Math.PI / 180;
        private double _slice_leadout_angle = 0.5 * Math.PI / 180;

        public double Tool_d                                      { set { _tool_r = value / 2.0;}}
        public double General_tolerance                           { set { _general_tolerance = value; } }
        public double Margin                                      { set { _margin = value; } }
        public double Max_ted                                     { set { _max_ted = value; } }
        public double Min_ted                                     { set { _min_ted = value; } }
        public double Slice_leadin_angle                          { set { _slice_leadin_angle = value; } }
        public double Slice_leadout_angle                         { set { _slice_leadout_angle = value; } }
        public Point2F Startpoint                                 { set { _startpoint = value; } }
        public RotationDirection Mill_direction                   { set { _dir = value; } }
        public bool Should_smooth_chords                          { set { _should_smooth_chords = value; }}

        private RotationDirection _initial_dir
        {
            // since unknown is 'dont care', CW is ok
            get { return _dir != RotationDirection.Unknown ? _dir : RotationDirection.CW; }
        }

        private double _min_passable_mic_radius
        {
            get { return 0.1 * _tool_r; } // 5 % of tool diameter is seems to be ok
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

            plist.AddRange(sample_curve(_outline, _tool_r / 10));
            foreach (Polyline p in _islands)
                plist.AddRange(sample_curve(p, _tool_r / 10));

            Logger.log("got {0} points", plist.Count);

            double[] xs = new double[plist.Count + 1];
            double[] ys = new double[plist.Count + 1];

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
            xs[plist.Count] = xs[lb_idx];
            ys[plist.Count] = ys[lb_idx] - width / 2;

            min_x -= VORONOI_MARGIN;
            max_x += VORONOI_MARGIN;
            min_y -= VORONOI_MARGIN + width / 2;
            max_y += VORONOI_MARGIN;

            List<GraphEdge> edges = new Voronoi(_general_tolerance).generateVoronoi(xs, ys, min_x, max_x, min_y, max_y);

            Logger.log("voronoi partitioning completed. got {0} edges", edges.Count);

            List<Line2F> inner_segments = new List<Line2F>();

            foreach (GraphEdge e in edges)
            {
                Line2F seg = new Line2F(e.x1, e.y1, e.x2, e.y2);
                if (seg.Length() < double.Epsilon) continue;    // extra small segment, discard
                if (! is_line_inside_region(seg, ANALIZE_INNER_INTERSECTIONS)) continue;
                inner_segments.Add(seg);
            }

            Logger.log("got {0} inner segments", inner_segments.Count);

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
            return radius - _tool_r - _margin;
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

        // find the path from src to dst thru least commong ancestor
        // path is going via the slices centers and excludes the src and dst themselves
        private List<Point2F> calc_lca_path(Slice dst, Slice src)
        {
            List<Point2F> path = new List<Point2F>();

            List<Slice> src_ancestry = new List<Slice>();
            List<Slice> dst_ancestry = new List<Slice>();

            for (Slice s = src.Parent; s != null; s = s.Parent)
                src_ancestry.Insert(0, s);

            for (Slice s = dst.Parent; s != null; s = s.Parent)
                dst_ancestry.Insert(0, s);

            int lca;
            for (lca = 0; lca < Math.Min(src_ancestry.Count, dst_ancestry.Count); lca++)
            {
                if (src_ancestry[lca] != dst_ancestry[lca])
                    break;
            }

            if (lca == 0)
            {
                ;   // one of the slices must be the root (no ancestry)
            }
            else
            {
                lca -= 1;   // the first diverging slices in ancestries were detected, lca is the last same slice, so -1
            }

            // now lca contains the lca of branches
            // collect path up from src to lca and down to dst
            for (int i = src_ancestry.Count - 1; i > lca; i--)
                path.Add(src_ancestry[i].Center);

            for (int i = lca; i < dst_ancestry.Count - 1; i++)
                path.Add(dst_ancestry[i].Center);

            return path;
        }

        private Pocket_path_item trace_branch_switch(Slice dst, Slice src, T4 ready_slices)
        {
            Point2F current = src.End;
            Point2F end = dst.Start;

            if (dst.Parent == src)  // simple continuation
                return connect_slices(dst, src);

            // follow the lca path, while looking for a shortcut to reduce travel time
            // TODO: skip parts of path to reduce travel even more
            Pocket_path_item p = new Pocket_path_item(Pocket_path_item_type.BRANCH_ENTRY);

            List<Point2F> path = calc_lca_path(dst, src);
            p.Add(current);
            foreach (Point2F pt in path)
            {
                if (may_shortcut(current, end, ready_slices))
                    break;
                current = pt;
                p.Add(current);
            }
            p.Add(end);

            return p;
        }

        private Pocket_path_item trace_return_to_base(Slice root_slice, Slice last_slice, T4 all_slices)
        {
            Point2F current = last_slice.End;
            Point2F end = root_slice.Center;

            Pocket_path_item p = new Pocket_path_item(Pocket_path_item_type.RETURN_TO_BASE);

            List<Point2F> path = new List<Point2F>();

            if (last_slice != root_slice)
            {
                for (Slice s = last_slice.Parent; s != root_slice; s = s.Parent)
                    path.Add(s.Center);
            }

            p.Add(current);
            foreach (Point2F pt in path)
            {
                if (may_shortcut(current, end, all_slices))
                    break;
                current = pt;
                p.Add(current);
            }
            p.Add(end);

            return p;
        }

        private void trace_branch(Branch branch, T4 ready_slices, ref Slice last_slice)
        {
            Slice parent_slice = null;

            if (branch.Curve.Points.Count == 0)
                throw new Exception("branch with the empty curve");

            Point2F start_pt = branch.Curve.Start;
            double start_radius = get_mic_radius(start_pt);

            if (branch.Parent != null)
            {
                // non-initial slice
                parent_slice = branch.Get_upstream_slice();
                if (parent_slice == null)
                    throw new Exception("parent slice is null - shouldn't be");

                if (last_slice == null)
                    throw new Exception("last slice is null - shouldn't be");
            }
            else
            {
                Slice s = new Slice(start_pt, start_radius, _initial_dir);
                branch.Slices.Add(s);
                insert_slice_in_t4(ready_slices, s);
                parent_slice = s;
                last_slice = s;
            }

            double left = 0;
            while (true)
            {
                Slice candidate = null;

                double right = 1.0;

                while (true)
                {
                    double mid = (left + right) / 2;

                    Point2F pt = branch.Curve.Get_parametric_pt(mid);

                    double radius = get_mic_radius(pt);

                    if (radius < _min_passable_mic_radius)
                    {
                        right = mid;    // assuming the branch is always starting from passable mics, so it's a narrow channel and we should be more conservative, go left
                    }
                    else
                    {
                        Slice s = new Slice(parent_slice, pt, radius, _dir, _tool_r, last_slice);

                        if (s.Max_ted == 0)  // no intersections, two possible cases
                        {
                            if (s.Dist <= 0)        // balls are inside each other, go right
                                left = mid;
                            else
                                right = mid;        // balls are spaced too far, go left
                        }
                        else    // intersection
                        {
                            // XXX: is this candidate is better than the last ?
                            candidate = s;
                            List<Slice> colliding_slices = find_colliding_slices(candidate, ready_slices);
                            candidate.Refine(colliding_slices, _tool_r, _tool_r);

                            if (candidate.Max_ted > _max_ted)
                            {
                                right = mid;        // overshoot, go left
                            }
                            else if ((_max_ted - candidate.Max_ted) / _max_ted > TED_TOLERANCE_PERCENTAGE)
                            {
                                left = mid;         // undershoot outside the strict TED tolerance, go right
                            }
                            else
                            {
                                left = mid;         // good slice inside the tolerance, stop search
                                break;
                            }
                        }
                    }

                    Point2F other = branch.Curve.Get_parametric_pt(left == mid ? right : left);
                    if (pt.DistanceTo(other) < _general_tolerance)
                    {
                        left = mid;                 // range has shrinked, stop search
                        break;
                    }
                }

                if (candidate == null)
                {
//                    Logger.log("no suitable candidates found at all. stopping slicing the branch");
                    return;
                }

                // discard slice if outside the specified min TED
                if (candidate.Max_ted < _min_ted)
                    return;

                double err = (candidate.Max_ted - _max_ted) / _max_ted;
                // discard slice if outside the final allowed percentage
                if (err > FINAL_ALLOWED_TED_OVERSHOOT_PERCENTAGE)
                {
                    if (DROP_BRANCH_ON_OVERSHOOT)
                    {
                        Logger.warn("failed to create slice within stepover limit. stopping slicing the branch");
                        return;
                    }
                    else
                    {
                        Logger.warn("forced to produce slice with stepover = {0}. It's a {1}% more than the normal stepover. Please inspect slice at {2}.",
                                                             candidate.Max_ted / (_tool_r * 2),
                                                             err * 100,
                                                             candidate.Start.ToString());
                    }
                }

                // if last slice was a root slice, adjust root slice startpoint to remove extra travel and
                // append leadout to the candindate. leadin is not needed, since join with root will be exact
                // otherwise append both leadin and leadout
                if (last_slice.Parent == null)
                {
                    last_slice.Change_startpoint(candidate.Start);
                    candidate.Append_leadin_and_leadout(0, _slice_leadout_angle);
                }
                else
                {
                    candidate.Append_leadin_and_leadout(_slice_leadin_angle, _slice_leadout_angle);
                }

                // generate branch entry after finding the first valid slice (before populating ready slices)
                if (branch.Slices.Count == 0)
                    branch.Entry = trace_branch_switch(candidate, last_slice, ready_slices);

                branch.Slices.Add(candidate);
                insert_slice_in_t4(ready_slices, candidate);
                parent_slice = candidate;
                last_slice = candidate;
            }
        }

        private void build_branch(Branch me, Segpool pool)
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
                build_branch(b, pool);

                if (b.Deep_distance() > _general_tolerance) // attach only 'long enough'
                    me.Children.Add(b);
                else
                    Logger.log("skipping short branch");
            }
            // prefer a shortest branch
            me.Children.Sort((a, b) => a.Deep_distance().CompareTo(b.Deep_distance()));
        }

        private Branch build_tree(List<Line2F> segments)
        {
            Segpool pool = new Segpool(segments.Count, _general_tolerance);
            Branch root = new Branch(null);
            Point2F tree_start = Point2F.Undefined;

            Logger.log("analyzing segments");

            // a lot of stuff going on here.
            // segments are analyzed for mic radius from both ends. passed segmens are inserted in segpool
            // hashed by one or both endpoints. if endpoint is not hashed, segment wouldn't be followed
            // from that side, preventing formation of bad tree.
            // segments are connected later in a greedy fashion, hopefully forming a mat covering all
            // pocket.
            // simultaneously we are looking for the tree root point - automatic, as a point with the largest mic,
            // or manually, as a mat segment nearest to the user specified start point.
            if (_startpoint.IsUndefined)
            {
                // automatic startpoint, choose the start segment - the one with the largest mic
                double max_r = double.MinValue;

                foreach (Line2F line in segments)
                {
                    double r1 = get_mic_radius(line.p1);
                    double r2 = get_mic_radius(line.p2);

                    if (r1 >= _min_passable_mic_radius)
                    {
                        pool.Add(line, false);
                        if (r1 > max_r)
                        {
                            max_r = r1;
                            tree_start = line.p1;
                        }
                    }
                    if (r2 >= _min_passable_mic_radius)
                    {
                        pool.Add(line, true);
                        if (r2 > max_r)
                        {
                            max_r = r2;
                            tree_start = line.p2;
                        }
                    }
                }
            }
            else
            {
                // manual startpoint, seek the segment with the closest end to startpoint
                if (! is_line_inside_region(new Line2F(_startpoint, _startpoint), false))
                {
                    Logger.warn("startpoint is outside the pocket");
                    return null;
                }
                if (get_mic_radius(_startpoint) < _min_passable_mic_radius)
                {
                    Logger.warn("startpoint radius < tool radius");
                    return null;
                }

                // insert startpoing to root poly, it would be connected to seg_start later
                root.Curve.Add(_startpoint);

                double min_dist = double.MaxValue;

                foreach (Line2F line in segments)
                {
                    double r1 = get_mic_radius(line.p1);
                    double r2 = get_mic_radius(line.p2);

                    if (r1 >= _min_passable_mic_radius)
                    {
                        pool.Add(line, false);
                        double dist = _startpoint.DistanceTo(line.p1);
                        if (dist < min_dist && is_line_inside_region(new Line2F(_startpoint, line.p1), true))
                        {
                            min_dist = dist;
                            tree_start = line.p1;
                        }
                    }
                    if (r2 >= _min_passable_mic_radius)
                    {
                        pool.Add(line, true);
                        double dist = _startpoint.DistanceTo(line.p2);
                        if (dist < min_dist && is_line_inside_region(new Line2F(_startpoint, line.p2), true))
                        {
                            min_dist = dist;
                            tree_start = line.p2;
                        }
                    }
                }
            }

            if (tree_start.IsUndefined)
            {
                Logger.warn("failed to choose tree start point");
                return null;
            }

            Logger.log("done analyzing segments");
            Logger.log("got {0} hashes", pool.N_hashes);

            root.Curve.Add(tree_start);
            build_branch(root, pool);
            return root;
        }

        private void insert_slice_in_t4(T4 t4, Slice slice)
        {
            Point2F min = Point2F.Undefined;
            Point2F max = Point2F.Undefined;
            slice.Get_ball_extrema(ref min, ref max);
            T4_rect rect = new T4_rect(min.X, min.Y, max.X, max.Y);
            t4.Add(rect, slice);
        }

        private void insert_polyline_in_t4(T4 t4, Polyline p)
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
                Line2F insects = s.Ball.LineIntersect(path, _general_tolerance);

                if (insects.p1.IsUndefined && insects.p2.IsUndefined)
                {
                    // no intersections: check if whole path lay inside the circle
                    if (   a.DistanceTo(s.Center) < s.Radius + _general_tolerance
                        && b.DistanceTo(s.Center) < s.Radius + _general_tolerance)
                        return true;
                }
                else if (insects.p1.IsUndefined || insects.p2.IsUndefined)
                {
                    // single intersection. one of the path ends must be inside the circle, otherwise it is a tangent case
                    // and should be ignored
                    if (a.DistanceTo(s.Center) < s.Radius + _general_tolerance)
                    {
                        running_collides.Add(s);
                    }
                    else if (b.DistanceTo(s.Center) < s.Radius + _general_tolerance)
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

                if (running_collides.Count == 0 && (ins.Key + _general_tolerance < a.DistanceTo(b)))
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

        private Pocket_path_item connect_slices_with_biarc(Slice dst, Slice src)
        {
            Point2F start = src.End;
            Point2F end = dst.Start;
            // unit normals to points
            Vector2d vn_start = new Vector2d(src.Center, start).Unit();
            Vector2d vn_end = new Vector2d(dst.Center, end).Unit();
            // tangents to points
            Vector2d vt_start;
            Vector2d vt_end;
            if (src.Dir == RotationDirection.CW)
                vt_start = new Vector2d(vn_start.Y, -vn_start.X);
            else
                vt_start = new Vector2d(-vn_start.Y, vn_start.X);

            if (dst.Dir == RotationDirection.CW)
                vt_end = new Vector2d(vn_end.Y, -vn_end.X);
            else
                vt_end = new Vector2d(-vn_end.Y, vn_end.X);

            Biarc2d biarc = new Biarc2d(start, vt_start, end, vt_end);

            if (!is_biarc_inside_ball(biarc, src.Ball))
                return null;

            Pocket_path_item path = new Pocket_path_item(Pocket_path_item_type.SMOOTH_CHORD);
            path.Add(biarc, _general_tolerance);
            return path;
        }

        private Pocket_path_item connect_slices_with_chord(Slice dst, Slice src)
        {
            Pocket_path_item path = new Pocket_path_item(Pocket_path_item_type.CHORD);
            path.Add(src.End);
            path.Add(dst.Start);
            return path;
        }

        private bool is_biarc_inside_ball(Biarc2d biarc, Circle2F ball)
        {
            if (biarc.Pm.DistanceTo(ball.Center) > ball.Radius)
                return false;

            Point2F start = biarc.P1;
            Point2F end = biarc.P2;
            Line2F insects;

            if (biarc.Seg1 is Line2F)
                insects = ball.LineIntersect((Line2F)biarc.Seg1);
            else
                insects = ((Arc2F)biarc.Seg1).CircleIntersect(ball);

            if ((! insects.p1.IsUndefined) && insects.p1.DistanceTo(start) < _general_tolerance)
                insects.p1 = Point2F.Undefined;
            if ((!insects.p2.IsUndefined) && insects.p2.DistanceTo(start) < _general_tolerance)
                insects.p2 = Point2F.Undefined;

            if (!(insects.p1.IsUndefined && insects.p2.IsUndefined))
                return false;

            if (biarc.Seg2 is Line2F)
                insects = ball.LineIntersect((Line2F)biarc.Seg2);
            else
                insects = ((Arc2F)biarc.Seg2).CircleIntersect(ball);

            if ((!insects.p1.IsUndefined) && insects.p1.DistanceTo(end) < _general_tolerance)
                insects.p1 = Point2F.Undefined;
            if ((!insects.p2.IsUndefined) && insects.p2.DistanceTo(end) < _general_tolerance)
                insects.p2 = Point2F.Undefined;

            if (!(insects.p1.IsUndefined && insects.p2.IsUndefined))
                return false;

            return true;
        }

        private Pocket_path_item connect_slices(Slice dst, Slice src)
        {
            Pocket_path_item path;

            // do not emit biarcs if distance is too small ( < 5 % of the tool diameter)
            if (_should_smooth_chords && src.End.DistanceTo(dst.Start) > _tool_r * 0.1)
            {
                path = connect_slices_with_biarc(dst, src);
                if (path != null)
                    return path;
                // fallback to the straight chord
                Logger.warn("biarc is outside the slice, replacing with chord");
            }

            return connect_slices_with_chord(dst, src);
        }

        private Pocket_path generate_path(List<Branch> traverse, T4 all_slices)
        {
            Slice last_slice = null;

            Pocket_path path = new Pocket_path();

            Slice root_slice = traverse[0].Slices[0];

            // emit spiral toolpath for root
            Pocket_path_item spiral = new Pocket_path_item(Pocket_path_item_type.SPIRAL);
            foreach (Biarc2d biarc in Spiral_generator.Gen_archimedean_spiral(root_slice.Center, root_slice.Start, _max_ted, _initial_dir))
                spiral.Add(biarc, _general_tolerance);
            path.Add(spiral);

            for (int bidx = 0; bidx < traverse.Count; bidx++)
            {
                Branch b = traverse[bidx];

                if (_should_emit_debug_medial_axis)
                {
                    Pocket_path_item mat = new Pocket_path_item(Pocket_path_item_type.DEBUG_MEDIAL_AXIS);
                    mat.Add(b.Curve);
                    path.Add(mat);
                }

                // emit branch entry path
                if (b.Entry != null)
                    path.Add(b.Entry);

                for (int sidx = 0; sidx < b.Slices.Count; sidx++)
                {
                    Slice s = b.Slices[sidx];

                    // connect following branch slices with chords
                    if (sidx > 0)
                        path.Add(connect_slices(s, last_slice));

                    // emit segments
                    for (int segidx = 0; segidx < s.Segments.Count; segidx++)
                    {
                        // connect segments
                        if (segidx > 0)
                        {
                            Pocket_path_item shortcut = new Pocket_path_item(Pocket_path_item_type.SLICE_SHORTCUT);
                            shortcut.Add(s.Segments[segidx - 1].P2);
                            shortcut.Add(s.Segments[segidx].P1);
                            path.Add(shortcut);
                        }

                        Pocket_path_item slice = new Pocket_path_item(Pocket_path_item_type.SLICE);
                        slice.Add(s.Segments[segidx], _general_tolerance);
                        path.Add(slice);
                    }
                    last_slice = s;
                }
            }

            path.Add(trace_return_to_base(root_slice, last_slice, all_slices));

            return path;
        }

        public Pocket_path run()
        {
            if (_dir == RotationDirection.Unknown && _should_smooth_chords)
                throw new Exception("smooth chords are not allowed for the variable mill direction");

            List<Line2F> mat_lines = get_mat_segments();

            Logger.log("building tree");
            Branch root = build_tree(mat_lines);
            if (root == null)
            {
                Logger.warn("failed to build tree");
                return null;
            }

            List<Branch> traverse = root.Df_traverse();

            T4 ready_slices = new T4(_reg_t4.Rect);
            Slice last_slice = null;

            Logger.log("generating slices");
            foreach (Branch b in traverse)
                trace_branch(b, ready_slices, ref last_slice);

            Logger.log("generating path");
            return generate_path(traverse, ready_slices);
        }

        public Pocket_generator(Polyline outline, Polyline[] islands)
        {
            _outline = outline;
            _islands = islands;

            Point3F min = Point3F.Undefined;
            Point3F max = Point3F.Undefined;

            _outline.GetExtrema(ref min, ref max);

            _reg_t4 = new T4(new T4_rect(min.X - 1, min.Y - 1, max.X + 1, max.Y + 1));

            insert_polyline_in_t4(_reg_t4, _outline);
            foreach (Polyline island in _islands)
                insert_polyline_in_t4(_reg_t4, island);
        }
    }
}
