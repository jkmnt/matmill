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

        private RotationDirection _initial_dir
        {
            // since unknown is 'dont care', CW is ok
            get { return _dir != RotationDirection.Unknown ? _dir : RotationDirection.CW; }
        }

        private double _min_passable_mic_radius
        {
            get { return 0.1 * _tool_r; } // 5 % of tool diameter is seems to be ok
        }

        private double get_mic_radius(Point2F pt)
        {
            return _topo.Get_dist_to_wall(pt) - _tool_r - _margin;
        }

        private Pocket_path generate_path(List<Branch> traverse, Slicer slicer)
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

                Ordered_slice s = b.Slices[0];

                if (s == root_slice)
                    gen.Append_root_slice(s);
                else if (s.Guide != null)
                    gen.Append_switch_slice(s, s.Guide);
                else
                    gen.Append_slice(s);

                for (int slice_idx = 1; slice_idx < b.Slices.Count; slice_idx++)
                {
                    s = b.Slices[slice_idx];
                    if (s.Guide != null)
                        gen.Append_switch_slice(s, s.Guide);
                    else
                        gen.Append_slice(s);
                }

                last_slice = s;
            }

            gen.Append_return_to_base(slicer.Gen_return_path());

            return gen.Path;
        }

        private double radius_getter(Point2F pt)
        {
            double radius = get_mic_radius(pt);
            if (radius < _min_passable_mic_radius) return 0;
            return radius;
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

            Slicer slicer = new Slicer(_topo.Bbox);
            slicer.Dir = _dir;
            slicer.General_tolerance = _general_tolerance;
            slicer.Initial_dir = _initial_dir;
            slicer.Max_ted = _max_ted;
            slicer.Min_ted = _min_ted;
            slicer.Slice_leadin_angle = _slice_leadin_angle;
            slicer.Slice_leadout_angle = _slice_leadout_angle;
            slicer.Tool_r = _tool_r;
            slicer.Get_radius = radius_getter;

            Logger.log("generating slices");
            slicer.Run(traverse);

            Logger.log("generating path");
            return generate_path(traverse, slicer);
        }

        public Pocket_generator(Polyline outline, Polyline[] islands)
        {
            _topo = new Topographer(outline, islands);
        }
    }
}
