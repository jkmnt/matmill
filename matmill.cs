using System;
using System.Collections.Generic;

using CamBam.CAD;
using CamBam.Geom;

using Geom;

namespace Matmill
{
    public class Pocket_generator
    {
        private readonly Topographer _topo;

        private double _general_tolerance = 0.001;
        private double _tool_r = 1.5;
        private double _margin = 0;
        private double _max_ted = 3.0 * 0.4;
        private double _min_ted = 3.0 * 0.1;
        private Point2F _startpoint = Point2F.Undefined;
        private RotationDirection _dir = RotationDirection.CW;
        private bool _should_smooth_chords = false;
        private double _slice_leadin_angle = 3 * Math.PI / 180;
        private double _slice_leadout_angle = 0.5 * Math.PI / 180;

        private bool EMIT_DEBUG_MEDIAL = true;

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

        private double _min_passable_mic_radius
        {
            get { return 0.1 * _tool_r; } // 5 % of tool diameter is seems to be ok
        }

        private double get_mic_radius(Point2F pt)
        {
            return _topo.Get_dist_to_wall(pt) - _tool_r - _margin;
        }

        private Sliced_path generate_path(Slice_sequence sequence)
        {
            Sliced_path_generator gen;

            if (!_should_smooth_chords)
                gen = new Sliced_path_generator(_general_tolerance);
            else
                gen = new Sliced_path_smooth_generator(_general_tolerance, 0.1 * _tool_r);

            gen.Append_spiral(sequence.Root_slice.Center, sequence.Root_slice.End, new Vector2d(), _max_ted, _tool_r, _dir == RotationDirection.Unknown ? RotationDirection.CCW : _dir);
            gen.Append_slice_sequence(sequence);

            return gen.Path;
        }

        private double radius_getter(Point2F pt)
        {
            double radius = get_mic_radius(pt);
            if (radius < _min_passable_mic_radius) return 0;
            return radius;
        }

        public Sliced_path Run()
        {
            if (_dir == RotationDirection.Unknown && _should_smooth_chords)
                throw new Exception("smooth chords are not allowed for the variable mill direction");

            Logger.log("building medial axis");

            Branch tree = new Branch(null);
            bool is_ok = _topo.Build_medial_tree(tree, _tool_r / 10, _general_tolerance, _startpoint, _min_passable_mic_radius + _tool_r + _margin);

            if (! is_ok)
            {
                Logger.warn("failed to build tree");
                return null;
            }

            Branch_slicer slicer = new Branch_slicer(_topo.Min, _topo.Max, _general_tolerance);
            slicer.Slice_leadin_angle = _slice_leadin_angle;
            slicer.Slice_leadout_angle = _slice_leadout_angle;
            slicer.Get_radius = radius_getter;

            Logger.log("generating slices");
            Slice_sequence sequence = slicer.Run(tree, _tool_r, _max_ted, _min_ted, _dir);

            Logger.log("generating path");
            Sliced_path path = generate_path(sequence);

            if (EMIT_DEBUG_MEDIAL)
            {
                foreach (Branch b in tree.Df_traverse())
                    path.Add(new Sliced_path_item(Sliced_path_item_type.DEBUG_MEDIAL_AXIS, b.To_polyline()));
            }

            return path;
        }

        public List<Polyline> Get_debug_medial_axis()
        {
            List<Polyline> result = new List<Polyline>();

            Branch tree = new Branch(null);
            bool is_ok = _topo.Build_medial_tree(tree, _tool_r / 10, _general_tolerance, _startpoint, _min_passable_mic_radius + _tool_r + _margin);

            if (is_ok)
            {
                foreach (Branch b in tree.Df_traverse())
                    result.Add(b.To_polyline());
            }

            return result;        
        }

        public Pocket_generator(Polyline outline, Polyline[] islands)
        {
            _topo = new Topographer(outline, islands);
        }
    }


    public class Engrave_generator
    {
        private const double TED_TOLERANCE_PERCENTAGE = 0.001;  // 0.1 %
        private const double FINAL_ALLOWED_TED_OVERSHOOT_PERCENTAGE = 0.03;  // 3 %

        private Polyline _poly;
        private Topographer _topo;

        private double _general_tolerance = 0.001;
        private double _tool_r = 1.5;
        private double _max_ted = 3.0 * 0.4;
        private Point2F _startpoint = Point2F.Undefined;
        private RotationDirection _dir = RotationDirection.CW;
        private bool _should_smooth_chords = false;
        private double _slice_leadin_angle = 3 * Math.PI / 180;
        private double _slice_leadout_angle = 0.5 * Math.PI / 180;
        private double _slice_radius = 1.5;

        public double Tool_d                                      { set { _tool_r = value / 2.0;}}
        public double General_tolerance                           { set { _general_tolerance = value; } }
        public double Max_ted                                     { set { _max_ted = value; } }
        public double Slice_radius                                { set { _slice_radius = value; } }
        public double Slice_leadin_angle                          { set { _slice_leadin_angle = value; } }
        public double Slice_leadout_angle                         { set { _slice_leadout_angle = value; } }
        public RotationDirection Mill_direction                   { set { _dir = value; } }
        public bool Should_smooth_chords                          { set { _should_smooth_chords = value; }}

        private int find_optimal_slice(Slice parent, Point2F pt, ref Slice _candidate)
        {
            Slice s = new Slice(parent, pt, _slice_radius, RotationDirection.CCW, _tool_r, Point2F.Undefined);

            if (s.Placement == Slice_placement.INSIDE_ANOTHER) return 1;    // go right
            if (s.Placement == Slice_placement.TOO_FAR) return -1;          // go left

            _candidate = s;

            if (s.Max_ted > _max_ted) return -1;                                            // overshoot, go left
            if ((_max_ted - s.Max_ted) / _max_ted > TED_TOLERANCE_PERCENTAGE) return 1;     // undershoot outside the strict TED tolerance, go right

            return 0;                                                                       // good slice inside the tolerance, stop search
        }

        // NOTE: funny way to calc it: try a lot of slices on a test line, approaching the best placement with the correct TED
        private double calc_optimal_step()
        {
            Branch branch = new Branch(null);
            branch.Add_point(new Point2F(0, 0));
            branch.Add_point(new Point2F(0, _tool_r * 2));

            Slice slice_a = new Slice(new Point2F(0, 0), _slice_radius, RotationDirection.CCW);
            Slice slice_b = null;

            double t = 0.0;
            branch.Bisect(pt => find_optimal_slice(slice_a, pt, ref slice_b), ref t, _general_tolerance);

            return slice_a.Center.DistanceTo(slice_b.Center);
        }

        private Sliced_path generate_path(Slice_sequence sequence)
        {
            Sliced_path_generator gen;

            if (!_should_smooth_chords)
                gen = new Sliced_path_generator(_general_tolerance);
            else
                gen = new Sliced_path_smooth_generator(_general_tolerance, 0.1 * _tool_r);

            // NOTE: this manipulations are to make the beginning of spiral tangent to the polyline
            object obj = _poly.GetSegment(0);
            Vector2d tangent = new Vector2d();
            if (obj is Line2F)
            {
                Line2F line = (Line2F)obj;
                tangent = new Vector2d(line.p1, line.p2);
            }
            else if (obj is Arc2F)
            {
                Arc2F arc = (Arc2F)obj;
                tangent = new Vector2d(arc.Center, arc.P1).Normal();
                if (arc.Direction == RotationDirection.CW)
                    tangent = tangent.Inverted();
            }

            gen.Append_spiral(sequence.Root_slice.Center, sequence.Root_slice.End, tangent, _max_ted, _tool_r, _dir == RotationDirection.Unknown ? RotationDirection.CCW : _dir);
            gen.Append_slice_sequence(sequence);

            return gen.Path;
        }

        public Sliced_path Run()
        {
            if (_dir == RotationDirection.Unknown && _should_smooth_chords)
                throw new Exception("smooth chords are not allowed for the variable mill direction");

            _topo = new Topographer(_poly, new Polyline[] { });

            double step = calc_optimal_step();
            List<Point2F> slice_centers = _topo.Get_samples_exact(step);

            Bypoint_slicer slicer = new Bypoint_slicer(_topo.Min, _topo.Max, _general_tolerance);
            slicer.Slice_leadin_angle = _slice_leadin_angle;
            slicer.Slice_leadout_angle = _slice_leadout_angle;

            Logger.log("generating slices");
            Slice_sequence sequence = slicer.Run(slice_centers, _tool_r, _slice_radius, _dir);

            Logger.log("generating path");
            return generate_path(sequence);
        }

        public Engrave_generator(Polyline poly)
        {
            _poly = poly;
        }
    }
}
