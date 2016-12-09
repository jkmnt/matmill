using System;
using System.Collections.Generic;

using CamBam.CAD;
using CamBam.Geom;

using Tree4;
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
        private const double TED_TOLERANCE_PERCENTAGE = 0.001;  // 0.1 %
        private const double FINAL_ALLOWED_TED_OVERSHOOT_PERCENTAGE = 0.03;  // 3 %

        private readonly Topographer _topo;

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

        private class Branch_tracer_context
        {
            private readonly T4 _ready_slices;

            public Slice Root_slice = null;
            public Slice Parent_slice = null;
            public Slice Last_slice = null;
            public Slice Candidate = null;

            private void insert_in_t4(Slice s)
            {
                Point2F min = Point2F.Undefined;
                Point2F max = Point2F.Undefined;
                s.Get_ball_extrema(ref min, ref max);
                T4_rect rect = new T4_rect(min.X, min.Y, max.X, max.Y);
                _ready_slices.Add(rect, s);
            }

            public List<Slice> Find_colliding_slices(Slice s)
            {
                Point2F min = Point2F.Undefined;
                Point2F max = Point2F.Undefined;
                s.Get_extrema(ref min, ref max);

                T4_rect rect = new T4_rect(min.X, min.Y, max.X, max.Y);
                return _ready_slices.Get_colliding_objects<Slice>(rect);
            }

            public List<Slice> Find_intersecting_slices(Point2F a, Point2F b)
            {
                T4_rect rect = new T4_rect(Math.Min(a.X, b.X),
                                           Math.Min(a.Y, b.Y),
                                           Math.Max(a.X, b.X),
                                           Math.Max(a.Y, b.Y));

                return _ready_slices.Get_colliding_objects<Slice>(rect);
            }

            public void Add_slice(Slice s)
            {
                if (Root_slice == null)
                    Root_slice = s;
                Parent_slice = s;
                Last_slice = s;
                insert_in_t4(s);
            }

            public Branch_tracer_context(T4_rect bbox)
            {
                _ready_slices = new T4(bbox);
            }
        }

        private double get_mic_radius(Point2F pt)
        {
            return _topo.Get_dist_to_wall(pt) - _tool_r - _margin;
        }

        private Pocket_path_item trace_branch_switch(Slice dst, Slice src, Branch_tracer_context ctx)
        {
            Point2F current = src.End;
            Point2F end = dst.Start;

            if (dst.Parent == src)  // simple continuation
                return connect_slices(dst, src);

            // follow the lca path, while looking for a shortcut to reduce travel time
            // TODO: skip parts of path to reduce travel even more
            Pocket_path_item p = new Pocket_path_item(Pocket_path_item_type.BRANCH_ENTRY);

            p.Add(current);
            foreach (Slice s in Slice_utils.Find_lca_path(dst, src))
            {
                if (may_shortcut(current, end, ctx))
                    break;
                current = s.Center;
                p.Add(current);
            }
            p.Add(end);

            return p;
        }

        private Pocket_path_item trace_return_to_base(Slice root_slice, Slice last_slice, Branch_tracer_context ctx)
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
                if (may_shortcut(current, end, ctx))
                    break;
                current = pt;
                p.Add(current);
            }
            p.Add(end);

            return p;
        }

        private int evaluate_possible_slice(Branch_tracer_context ctx, Point2F pt)
        {
            double radius = get_mic_radius(pt);

            if (radius < _min_passable_mic_radius) return -1; // assuming the branch is always starting from passable mics, so it's a narrow channel and we should be more conservative, go left

            Slice s = new Slice(ctx.Parent_slice, pt, radius, _dir, _tool_r, ctx.Last_slice);

            if (s.Placement != Slice_placement.NORMAL)
            {
                if (s.Placement == Slice_placement.INSIDE_ANOTHER) return 1;    // go right
                if (s.Placement == Slice_placement.TOO_FAR) return -1;          // go left

                throw new Exception("unknown slice placement");
            }
            // intersection
            // XXX: is this candidate is better than the last ?
            ctx.Candidate = s;
            s.Refine(ctx.Find_colliding_slices(s), _tool_r, _tool_r);

            if (s.Max_ted > _max_ted) return -1;                                            // overshoot, go left
            if ((_max_ted - s.Max_ted) / _max_ted > TED_TOLERANCE_PERCENTAGE) return 1;     // undershoot outside the strict TED tolerance, go right

            return 0;                                                                       // good slice inside the tolerance, stop search
        }

        private void trace_branch(Branch_tracer_context ctx, Branch branch)
        {
            if (branch.Curve.Points.Count == 0)
                throw new Exception("branch with the empty curve");

            if (branch.Parent != null)  // non-root slice
            {
                ctx.Parent_slice = branch.Get_upstream_slice();
            }
            else
            {
                Point2F start_pt = branch.Curve.Start;
                Slice s = new Slice(start_pt, get_mic_radius(start_pt), _initial_dir);
                branch.Slices.Add(s);
                ctx.Add_slice(s);
            }

            if (ctx.Parent_slice == null)
                throw new Exception("parent slice is null - shouldn't be");
            if (ctx.Last_slice == null)
                throw new Exception("last slice is null - shouldn't be");

            double t = 0;

            while (true)
            {
                ctx.Candidate = null;
                branch.Bisect(pt => evaluate_possible_slice(ctx, pt), ref t, _general_tolerance);
                Slice candidate = ctx.Candidate;

                if (candidate == null) return;
                if (candidate.Max_ted < _min_ted) return; // discard slice if outside the specified min TED

                // discard slice if outside the final allowed percentage
                double err = (candidate.Max_ted - _max_ted) / _max_ted;
                if (err > FINAL_ALLOWED_TED_OVERSHOOT_PERCENTAGE)
                {
                    Logger.warn("failed to create slice within stepover limit. stopping slicing the branch");
                    return;
                }
                // if last slice was a root slice, adjust root slice startpoint to remove extra travel and
                // append leadout to the candindate. leadin is not needed, since join with root will be exact
                // otherwise append both leadin and leadout
                if (ctx.Last_slice.Parent == null)
                {
                    ctx.Last_slice.Change_startpoint(candidate.Start);
                    candidate.Append_leadin_and_leadout(0, _slice_leadout_angle);
                }
                else
                {
                    candidate.Append_leadin_and_leadout(_slice_leadin_angle, _slice_leadout_angle);
                }

                // generate branch entry after finding the first valid slice (before populating ready slices)
                if (branch.Slices.Count == 0)
                    branch.Entry = trace_branch_switch(candidate, ctx.Last_slice, ctx);

                branch.Slices.Add(candidate);
                ctx.Add_slice(candidate);
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

        private bool may_shortcut(Point2F a, Point2F b, Branch_tracer_context ctx)
        {
            return may_shortcut(a, b, ctx.Find_intersecting_slices(a, b));
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

        private Pocket_path generate_path(List<Branch> traverse, Branch_tracer_context ctx)
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

            path.Add(trace_return_to_base(root_slice, last_slice, ctx));

            return path;
        }

        public Pocket_path run()
        {
            if (_dir == RotationDirection.Unknown && _should_smooth_chords)
                throw new Exception("smooth chords are not allowed for the variable mill direction");

            Logger.log("building medial axis");

            Branch root = _topo.Get_medial_axis(_tool_r / 10, _general_tolerance, _startpoint, _min_passable_mic_radius + _tool_r + _margin);

            if (root == null)
            {
                Logger.warn("failed to build tree");
                return null;
            }

            List<Branch> traverse = root.Df_traverse();

            Branch_tracer_context tracer_ctx = new Branch_tracer_context(_topo.Bbox);

            Logger.log("generating slices");
            foreach (Branch b in traverse)
                trace_branch(tracer_ctx, b);

            Logger.log("generating path");
            return generate_path(traverse, tracer_ctx);
        }

        public Pocket_generator(Polyline outline, Polyline[] islands)
        {
            _topo = new Topographer(outline, islands);
        }
    }
}
