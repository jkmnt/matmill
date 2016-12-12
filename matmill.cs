using System;
using System.Collections.Generic;

using CamBam.CAD;
using CamBam.Geom;

using Tree4;

namespace Matmill
{

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

        private List<Point2F> trace_branch_entry(Slice dst, Slice src, Branch_tracer_context ctx)
        {
            if (dst.Parent == src)  // simple continuation
                return null;

            // follow the lca path, while looking for a shortcut to reduce travel time
            // TODO: skip parts of path to reduce travel even more
            List<Point2F> knots = new List<Point2F>();

            Point2F current = src.End;
            Point2F end = dst.Start;

            foreach (Slice s in Slice_utils.Find_lca_path(dst, src))
            {
                if (may_shortcut(current, end, ctx))
                    break;
                current = s.Center;
                knots.Add(current);
            }

            return knots;
        }

        private List<Point2F> trace_return_to_base(Slice root_slice, Slice last_slice, Branch_tracer_context ctx)
        {
            Point2F current = last_slice.End;
            Point2F end = root_slice.Center;

            List<Point2F> path = new List<Point2F>();

            if (last_slice != root_slice)
            {
                for (Slice s = last_slice.Parent; s != root_slice; s = s.Parent)
                {
                    if (may_shortcut(current, end, ctx))
                        break;
                    current = s.Center;
                    path.Add(current);
                }
            }

            path.Add(end);
            return path;
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
            ctx.Parent_slice = branch.Get_upstream_slice();

            if (ctx.Parent_slice == null)   // the very start of trace
            {
                Point2F start_pt = branch.Start;
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
                    Logger.log("changing startpoint of root slice");
                    ctx.Last_slice.Change_startpoint(candidate.Start);
                    candidate.Append_leadin_and_leadout(0, _slice_leadout_angle);
                }
                else
                {
                    candidate.Append_leadin_and_leadout(_slice_leadin_angle, _slice_leadout_angle);
                }

                // generate branch entry after finding the first valid slice (before populating ready slices)
                if (branch.Slices.Count == 0)
                    branch.Entry_path = trace_branch_entry(candidate, ctx.Last_slice, ctx);

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

        private Pocket_path generate_path(List<Branch> traverse, Branch_tracer_context ctx)
        {
            Pocket_path_generator gen;

            if (!_should_smooth_chords)
                gen = new Pocket_path_generator(_general_tolerance);
            else
                gen = new Pocket_path_smooth_generator(_general_tolerance, 0.1 * _tool_r);

            Slice root_slice = traverse[0].Slices[0];

            gen.Append_spiral(root_slice.Center, root_slice.End, _max_ted, _initial_dir);

            Slice last_slice = null;

            foreach (Branch b in traverse)
            {
                if (b.Slices.Count == 0)
                    continue;

                Slice s = b.Slices[0];

                if (s == root_slice)
                    gen.Append_root_slice(s);
                else if (s.Parent != last_slice)
                    gen.Append_switch_slice(s, b.Entry_path);
                else
                    gen.Append_slice(s);

                for (int slice_idx = 1; slice_idx < b.Slices.Count; slice_idx++)
                {
                    s = b.Slices[slice_idx];
                    gen.Append_slice(s);
                }

                last_slice = s;
            }

            List<Point2F> return_path = trace_return_to_base(root_slice, last_slice, ctx);
            gen.Append_return_to_base(return_path);

            return gen.Path;
        }

        public Pocket_path run()
        {
            if (_dir == RotationDirection.Unknown && _should_smooth_chords)
                throw new Exception("smooth chords are not allowed for the variable mill direction");

            Logger.log("building medial axis");

            Branch root = new Branch(null);
            bool is_ok = _topo.Build_medial_axis(root, _tool_r / 10, _general_tolerance, _startpoint, _min_passable_mic_radius + _tool_r + _margin);

            if (! is_ok)
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
