using System;
using System.Collections.Generic;

using CamBam.Geom;

namespace Matmill
{
    class Slicer
    {        
        public delegate double Get_radius_delegate(Point2F pt);

        private const double TED_TOLERANCE_PERCENTAGE = 0.001;  // 0.1 %
        private const double FINAL_ALLOWED_TED_OVERSHOOT_PERCENTAGE = 0.03;  // 3 %

        private double _general_tolerance;
        private Ballfield_topographer _topo;

        private double _min_ted;
        private double _max_ted;
        private double _tool_r;
        private RotationDirection _dir;

        public List<Slice> Sequence = new List<Slice>();

        public double Slice_leadin_angle = 3 * Math.PI / 180;
        public double Slice_leadout_angle = 0.5 * Math.PI / 180;

        public Slice Root_slice { get { return Sequence.Count > 0 ? Sequence[0] : null; } }
        public Slice Last_slice { get { return Sequence.Count > 0 ? Sequence[Sequence.Count - 1] : null; } }
        // NOTE: radius getter may return 0 if radius is too small or invalid
        public Get_radius_delegate Get_radius = x => 0;

        // XXX: refactor me
        private static List<Slice> find_lca_path(Slice dst, Slice src)
        {
            List<Slice> path = new List<Slice>();

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
                path.Add(src_ancestry[i]);

            for (int i = lca; i < dst_ancestry.Count - 1; i++)
                path.Add(dst_ancestry[i]);

            return path;
        }

        private List<Slice> find_colliding_slices(Slice s)
        {
            Point2F min = Point2F.Undefined;
            Point2F max = Point2F.Undefined;
            s.Get_extrema(ref min, ref max);

            return _topo.Get_colliding_objects<Slice>(min, max);
        }

        private List<Point2F> trace_branch_switch(Slice dst, Slice src)
        {
            // follow the lca path, while looking for a shortcut to reduce travel time
            // TODO: skip parts of path to reduce travel even more
            List<Point2F> knots = new List<Point2F>();

            Point2F current = src.End;
            Point2F end = dst.Start;

            foreach (Slice s in find_lca_path(dst, src))
            {
                if (_topo.Is_line_inside_region(new Line2F(current, end), _general_tolerance))
                    break;
                current = s.Center;
                knots.Add(current);
            }

            return knots;
        }

        private List<Point2F> trace_return_to_base()
        {
            Point2F current = Last_slice.End;
            Point2F end = Root_slice.Center;

            List<Point2F> path = new List<Point2F>();

            if (Last_slice != Root_slice)
            {
                for (Slice s = Last_slice.Parent; s != Root_slice; s = s.Parent)
                {
                    if (_topo.Is_line_inside_region(new Line2F(current, end), _general_tolerance))
                        break;
                    current = s.Center;
                    path.Add(current);
                }
            }
            path.Add(end);

            return path;
        }        

        private int evaluate_possible_slice(Slice parent, Point2F pt, ref Slice _candidate)
        {
            double radius = Get_radius(pt);

            if (radius <= 0) return -1; // assuming the branch is always starting from passable mics, so it's a narrow channel and we should be more conservative, go left

            Slice s = new Slice(parent, pt, radius, _dir, _tool_r, Last_slice != null ? Last_slice.End : Point2F.Undefined);

            if (s.Placement != Slice_placement.NORMAL)
            {
                if (s.Placement == Slice_placement.INSIDE_ANOTHER) return 1;    // go right
                if (s.Placement == Slice_placement.TOO_FAR) return -1;          // go left

                throw new Exception("unknown slice placement");
            }
            // intersection
            // XXX: is this candidate is better than the last ?
            _candidate = s;
            s.Refine(find_colliding_slices(s), _tool_r, _tool_r);

            if (s.Max_ted > _max_ted) return -1;                                            // overshoot, go left
            if ((_max_ted - s.Max_ted) / _max_ted > TED_TOLERANCE_PERCENTAGE) return 1;     // undershoot outside the strict TED tolerance, go right

            return 0;                                                                       // good slice inside the tolerance, stop search
        }

        private void trace_branch(Branch branch, Slice parent_slice)
        {
            double t = 0;

            while (true)
            {
                Slice new_slice = null;

                branch.Bisect(pt => evaluate_possible_slice(parent_slice, pt, ref new_slice), ref t, _general_tolerance);

                if (new_slice == null)
                    break;
                if (new_slice.Max_ted < _min_ted)   // discard slice if outside the specified min TED
                    break;

                // discard slice if outside the final allowed percentage
                double err = (new_slice.Max_ted - _max_ted) / _max_ted;
                if (err > FINAL_ALLOWED_TED_OVERSHOOT_PERCENTAGE)
                {
                    Logger.warn("failed to create slice within stepover limit. stopping slicing the branch");
                    break;
                }

                // if last slice was a root slice, adjust root slice startpoint to remove extra travel and
                // append leadout to the candindate. leadin is not needed, since join with root will be exact
                // otherwise append both leadin and leadout
                if (Last_slice == Root_slice)
                {
                    Logger.log("changing startpoint of root slice");
                    Root_slice.Change_startpoint(new_slice.Start);
                    new_slice.Append_leadin_and_leadout(0, Slice_leadout_angle);
                }
                else
                {
                    new_slice.Append_leadin_and_leadout(Slice_leadin_angle, Slice_leadout_angle);
                }

                // generate guide move if this slice is not a simple continuation
                if (parent_slice != Last_slice)
                    new_slice.Guide = trace_branch_switch(new_slice, Last_slice);

                Sequence.Add(new_slice);
                _topo.Add(new_slice.Ball, new_slice);
                parent_slice = new_slice;
            }

            // need to go deeper
            foreach (Branch b in branch.Children)
                trace_branch(b, parent_slice);
        }

        public List<Point2F> Gen_return_path()
        {
            return trace_return_to_base();
        }

        public void Run(Branch root, double tool_r, double max_ted, double min_ted, RotationDirection dir)
        {
            _min_ted = min_ted;
            _max_ted = max_ted;
            _tool_r = tool_r;
            _dir = dir;

            _topo = new Ballfield_topographer(_topo.Min, _topo.Max);
            Sequence.Clear();

            Slice root_slice = new Slice(root.Start, Get_radius(root.Start), _dir == RotationDirection.Unknown ? RotationDirection.CCW : _dir);

            Sequence.Add(root_slice);
            _topo.Add(root_slice.Ball, root_slice);

            trace_branch(root, root_slice);
        }

        public Slicer(Point2F min, Point2F max, double general_tolerance)
        {
            _topo = new Ballfield_topographer(min, max);
            _general_tolerance = general_tolerance;
        }
    }
}
