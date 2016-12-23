using System;

using System.Collections.Generic;

using CamBam.CAD;
using CamBam.Geom;

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
        //private bool _should_emit_debug_medial_axis = false;
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

            gen.Append_spiral(sequence.Root_slice.Center, sequence.Root_slice.End, _max_ted, _dir == RotationDirection.Unknown ? RotationDirection.CCW : _dir);
            gen.Append_slice_sequence(sequence);            

            return gen.Path;
        }

        private double radius_getter(Point2F pt)
        {
            double radius = get_mic_radius(pt);
            if (radius < _min_passable_mic_radius) return 0;
            return radius;
        }

        public Sliced_path run()
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
            return generate_path(sequence);
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
        public Point2F Startpoint                                 { set { _startpoint = value; } }
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

        private static Polyline adjust_startpoint(Polyline poly, Point2F startpoint)
        {
            if (startpoint.IsUndefined)
                return poly;

            if (poly.Closed)
            {
                int seg = 0;
                double min_dist = double.MaxValue;

                for (int i = 0; i < poly.Points.Count; i++)
                {
                    Point2F pt = (Point2F)poly.Points[i].Point;
                    double dist = pt.DistanceTo(startpoint);
                    if (dist <= min_dist)
                    {
                        min_dist = dist;
                        seg = i;
                    }
                }
                if (seg != 0)
                    poly = poly.ToNewStartPoint(seg);
            }
            else
            {
                Point2F start = (Point2F)poly.FirstPoint;
                Point2F end = (Point2F)poly.LastPoint;

                if (startpoint.DistanceTo(end) < startpoint.DistanceTo(start))
                {
                    poly = new Polyline(poly);
                    poly.Reverse();
                }
            }

            return poly;
        }

        private Sliced_path generate_path(Slice_sequence sequence)
        {
            Sliced_path_generator gen;

            if (!_should_smooth_chords)
                gen = new Sliced_path_generator(_general_tolerance);
            else
                gen = new Sliced_path_smooth_generator(_general_tolerance, 0.1 * _tool_r);

            gen.Append_spiral(sequence.Root_slice.Center, sequence.Root_slice.End, _max_ted, _dir == RotationDirection.Unknown ? RotationDirection.CCW : _dir);
            gen.Append_slice_sequence(sequence);            

            return gen.Path;
        }

        public Sliced_path run()
        {
            if (_dir == RotationDirection.Unknown && _should_smooth_chords)
                throw new Exception("smooth chords are not allowed for the variable mill direction");

            _poly = adjust_startpoint(_poly, _startpoint);
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
