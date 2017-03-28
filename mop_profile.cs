using System;
using System.Reflection;
using System.Collections.Generic;
using System.ComponentModel;
using System.Drawing;
using System.Xml.Serialization;

using CamBam;
using CamBam.CAD;
using CamBam.CAM;
using CamBam.Geom;
using CamBam.UI;
using CamBam.Values;

using Matmill;

namespace Trochomops
{
    [Serializable]
    public class MOPTrochoprof : Trochomop, IIcon
    {
        protected CBValue<InsideOutsideOptions> _cut_side;
        protected CBValue<double> _cut_width;
        protected CBValue<bool> _should_overcut_corners;

        protected bool _should_clear_inside_corners;
        protected double _min_stepover_percentage = 0.9;

        //--- invisible and non-serializable properties

        [XmlIgnore, Browsable(false)]
        public override string MOPTypeName
        {
            get { return "TrochoProf"; }
        }

        [XmlIgnore, Browsable(false)]
        public Image ActiveIconImage
        {
            get { return resources.cam_trochoprof1;}
        }

        [XmlIgnore, Browsable(false)]
        public string ActiveIconKey
        {
            get { return "cam_trochoprof1"; }
        }

        [XmlIgnore, Browsable(false)]
        public Image InactiveIconImage
        {
            get { return resources.cam_trochoprof0;}
        }

        [XmlIgnore, Browsable(false)]
        public string InactiveIconKey
        {
            get { return "cam_trochoprof0"; }
        }

        //--- visible parameters which may be styled

		[
            CBKeyValue,
            Category("(General)"),
            DefaultValue(typeof(CBValue<InsideOutsideOptions>), "Default"),
            Description("Whether to cut Inside or Outside the selected shapes.\r\nFor open shapes the point order determines which side of the line to cut."),
            DisplayName("Inside / Outside")
        ]
		public CBValue<InsideOutsideOptions> InsideOutside
		{
			get { return this._cut_side;  }
			set { this._cut_side = value; }
		}

        [
            Category("Step Over"),
            DefaultValue(typeof(CBValue<double>), "Default"),
            Description("The total width of the sliced cut.\r\n"+
                        "If 0 the width of the cut is 2 * tool diameter"),
            DisplayName("Cut Width")
        ]
        public CBValue<double> CutWidth
        {
        	get { return this._cut_width; }
        	set { this._cut_width = value; }
        }

        [
            Category("Options"),
            DefaultValue(typeof(CBValue<bool>), "Default"),
            Description("Add a move to cut corners otherwise too narrow for cutter."),
            DisplayName("Corner Overcut")
        ]
        public CBValue<bool> CornerOvercut
        {
            get { return this._should_overcut_corners;}
            set { this._should_overcut_corners = value;}
        }


        //--- our own new parameters. No reason to make them CBValues, since they couldn't be styled anyway

        [
            CBAdvancedValue,
            Category("Step Over"),
            DefaultValue(0.9),
            Description("Minimum allowed stepover as a percentage of the nominal stepover (0.1 - 0.9).\nUsed only if 'Clear Inside Corners' enabled"),
            DisplayName("Minimum Stepover")
        ]
        public double Min_stepover
        {
            get { return _min_stepover_percentage; }
            set
            {
                _min_stepover_percentage = value;

                if (value < 0.05 || value > 0.95)
                {
                    _min_stepover_percentage = Math.Max(Math.Min(0.95, value), 0.05);
                    base.redraw_parameters();
                }
            }
        }

        [
            Category("Options"),
            DefaultValue(false),
            Description("Adds a moves to clear inside corners."),
            DisplayName("Clear Inside Corners")
        ]
        public bool Clear_inside_corners
        {
        	get { return this._should_clear_inside_corners;}
        	set { this._should_clear_inside_corners = value;}
        }

        private Vector2F calc_start_tangent(Polyline poly)
        {
            object obj = poly.GetSegment(0);
            Vector2F tangent = new Vector2F();
            if (obj is Line2F)
            {
                Line2F line = (Line2F)obj;
                tangent = new Vector2F(line.p1, line.p2);
            }
            else if (obj is Arc2F)
            {
                Arc2F arc = (Arc2F)obj;
                tangent = new Vector2F(arc.Center, arc.P1).Normal();
                if (arc.Direction == RotationDirection.CW)
                    tangent.Invert();
            }

            return tangent;
        }

        private Sliced_path gen_engrave_toolpath(Polyline poly, double width, Vector2F start_tangent)
        {
            Engrave_generator gen = new Engrave_generator(poly);

            gen.General_tolerance = is_inch_units() ? 0.001 / 25.4 : 0.001;
            gen.Tool_d = base.ToolDiameter.Cached;
            gen.Max_ted = base.ToolDiameter.Cached * _stepover.Cached;

            gen.Slice_radius = (width - base.ToolDiameter.Cached) / 2;

            gen.Spiral_tangent = start_tangent;

            if (_milling_direction.Cached == MillingDirectionOptions.Mixed || base.SpindleDirection.Cached == SpindleDirectionOptions.Off)
            {
                gen.Mill_direction = RotationDirection.Unknown; // means 'mixed' here
                gen.Should_smooth_chords = false;
            }
            else
            {
                int dir = (int)(base.SpindleDirection.Cached);
                if (_milling_direction.Cached == MillingDirectionOptions.Climb)
                    dir = -dir;
                gen.Mill_direction = (RotationDirection)dir;
                gen.Should_smooth_chords = _should_smooth_chords;
            }

            return gen.run();
        }

        private Sliced_path gen_pocket_toolpath(ShapeListItem shape, Point2F startpoint, Vector2F start_tangent)
        {
            Polyline outline;
            Polyline[] islands;

            if (shape.Shape is Polyline)
            {
                outline = (Polyline)shape.Shape;
                islands = new Polyline[] { };
            }
            else if (shape.Shape is CamBam.CAD.Region)
            {
                CamBam.CAD.Region reg = (CamBam.CAD.Region)shape.Shape;
                outline = reg.OuterCurve;
                islands = reg.HoleCurves;
            }
            else
            {
                return null;
            }

            Pocket_generator gen = new Pocket_generator(outline, islands);

            gen.General_tolerance = is_inch_units() ? 0.001 / 25.4 : 0.001;
            gen.Tool_d = base.ToolDiameter.Cached;
            gen.Max_ted = base.ToolDiameter.Cached * _stepover.Cached;
            gen.Min_ted = base.ToolDiameter.Cached * _stepover.Cached * _min_stepover_percentage;
            if (startpoint.IsUndefined)
                startpoint = (Point2F)outline.FirstPoint;
            gen.Startpoint = startpoint;
            gen.Startpoint_is_a_hint = true;
            gen.Margin = 0;
            gen.Spiral_tangent = start_tangent;

            if (_milling_direction.Cached == MillingDirectionOptions.Mixed || base.SpindleDirection.Cached == SpindleDirectionOptions.Off)
            {
                gen.Mill_direction = RotationDirection.Unknown; // means 'mixed' here
                gen.Should_smooth_chords = false;
            }
            else
            {
                int dir = (int)(base.SpindleDirection.Cached);
                if (_milling_direction.Cached == MillingDirectionOptions.Climb)
                    dir = -dir;
                gen.Mill_direction = (RotationDirection)dir;
                gen.Should_smooth_chords = _should_smooth_chords;
            }

            return gen.run();
        }

        private static Point2F lastpt(List<Sliced_path> trajectories)
        {
            if (trajectories.Count == 0)
                return Point2F.Undefined;
            Sliced_path last = trajectories[trajectories.Count - 1];
            if (last.Count == 0)
                return Point2F.Undefined;
            return (Point2F)last[last.Count - 1].LastPoint;
        }

        private static Polyline adjust_closed_startpoint(Polyline poly, Point2F startpoint)
        {
            Vector2F normal = Vector2F.Undefined;
            int nearest_seg = 0;
            Point3F nearest_pt = poly.GetNearestPoint(startpoint, ref normal, ref nearest_seg, true);

            if (nearest_seg >= 0)
            {
                int seg = poly.InsertPoint((Point2F)nearest_pt, (double)CamBamConfig.Defaults.GeneralTolerance);
                if (seg >= 0)
                    poly = poly.ToNewStartPoint(seg);
            }

            return poly;
        }

        private bool should_flip_opened_startpoint(Polyline poly, Point2F startpoint)
        {
            Point2F start = (Point2F)poly.FirstPoint;
            Point2F end = (Point2F)poly.LastPoint;

            return startpoint.DistanceTo(end) < startpoint.DistanceTo(start);
        }


        private List<Sliced_path> gen_coarse_profile(Polyline poly, bool is_inside, Point2F startpoint)
        {
            if (! startpoint.IsUndefined)
            {
                poly = new Polyline(poly);

                if (poly.Closed)
                {
                    poly = adjust_closed_startpoint(poly, startpoint);
                }
                else
                {
                    if (should_flip_opened_startpoint(poly, startpoint))
                    {
                        poly.Reverse();
                        is_inside = ! is_inside;    // preserve side if flipped !
                    }
                }
            }

            double cut_width = _cut_width.Cached;
            if (cut_width == 0)
                cut_width = base.ToolDiameter.Cached * 2;

            double offset = cut_width / 2 + base.RoughingClearance.Cached;

            if (is_inside)
                offset = -offset;


            List<Sliced_path> trajectories = new List<Sliced_path>();
            Polyline[] array;

            // special case - simulation of engrave. do not offset poly at all to allow exact follow of path
            if (Math.Abs(offset) < (double)CamBamConfig.Defaults.GeneralTolerance)
            {
                array = new Polyline[] { poly };
            }
            else
            {
                array = poly.CreateOffsetPolyline(offset, (double)CamBamConfig.Defaults.GeneralTolerance, _should_overcut_corners.Cached, false);
            }

            if (array == null)
                return trajectories;

            Vector2F start_tangent = calc_start_tangent(poly);

            foreach (Polyline p in array)
            {
                if (trajectories.Count != 0)
                    startpoint = lastpt(trajectories);

                Sliced_path toolpath = gen_engrave_toolpath(p, cut_width, start_tangent);
                if (toolpath != null)
                {
                    Traj_metainfo meta = new Traj_metainfo();
                    meta.Start_normal = new Vector2F((Point2F)toolpath[0].FirstPoint, (Point2F)poly.FirstPoint);
                    toolpath.Extension = meta;
                    trajectories.Add(toolpath);
                }
            }

            return trajectories;
        }


        private List<Sliced_path> gen_fine_profile(Polyline poly, bool is_inside, Point2F startpoint)
        {
            if (! startpoint.IsUndefined)
            {
                poly = adjust_closed_startpoint(poly, startpoint);
            }

            double cut_width = _cut_width.Cached;
            if (cut_width == 0)
                cut_width = base.ToolDiameter.Cached * 2;

            double clearance_offset = base.RoughingClearance.Cached;
            double offset = cut_width + clearance_offset;

            if (is_inside)
            {
                offset = -offset;
                clearance_offset = -clearance_offset;
            }

            Polyline[] profile_walls;

            if (clearance_offset != 0)
                profile_walls = poly.CreateOffsetPolyline(clearance_offset, (double)CamBamConfig.Defaults.GeneralTolerance, false, false);
            else
                profile_walls = new Polyline[] { poly };

            Polyline[] pseudo_walls = poly.CreateOffsetPolyline(offset, (double)CamBamConfig.Defaults.GeneralTolerance, false, false);

            ShapeList shapes = new ShapeList();
            shapes.ApplyTransformations = true;
            shapes.AddEntities(profile_walls);
            shapes.AddEntities(pseudo_walls);
            shapes = shapes.DetectRegions();

            List<Sliced_path> trajectories = new List<Sliced_path>();

            Vector2F start_tangent = calc_start_tangent(poly);

            foreach (ShapeListItem shape in shapes)
            {
                if (shape.Shape is Polyline && ! ((Polyline)shape.Shape).Closed)
                {
                    Logger.warn("got open polyline while offsetting profile. ignoring");
                    continue;
                }

                if (trajectories.Count != 0)
                    startpoint = lastpt(trajectories);

                Sliced_path toolpath = gen_pocket_toolpath(shape, startpoint, start_tangent);
                if (toolpath != null)
                {
                    Traj_metainfo meta = new Traj_metainfo();
                    meta.Start_normal = new Vector2F((Point2F)toolpath[0].FirstPoint, (Point2F)poly.FirstPoint);
                    toolpath.Extension = meta;
                    trajectories.Add(toolpath);
                }
            }

            return trajectories;
        }

        private List<Sliced_path> gen_profile(Polyline poly, bool is_inside, Point2F startpoint)
        {
            if (_should_clear_inside_corners)
            {
                if (poly.Closed)
                    return gen_fine_profile(poly, is_inside, startpoint);

                Logger.warn("corner clearing for open polylines is not implemented. proceeding without clearing");
            }
            return gen_coarse_profile(poly, is_inside, startpoint);
        }

        private List<Sliced_path> gen_profile(CamBam.CAD.Region region, bool is_inside, Point2F startpoint)
        {
            List<Sliced_path> trajectories = new List<Sliced_path>();

            trajectories.AddRange(gen_profile(region.OuterCurve, is_inside, startpoint));

            foreach (Polyline hole in region.HoleCurves)
            {
                if (trajectories.Count != 0)
                    startpoint = lastpt(trajectories);
                trajectories.AddRange(gen_profile(hole, !is_inside, startpoint));
            }

            return trajectories;
        }

        protected override void _GenerateToolpathsWorker()
        {
            try
            {
                base.reset_toolpaths();

                if (base.ToolDiameter.Cached == 0)
                {
                    Logger.err("tool diameter is zero");
                    base.MachineOpStatus = MachineOpStatus.Errors;
                    return;
                }

                if (_cut_width.Cached != 0 && _cut_width.Cached < base.ToolDiameter.Cached * 1.05)
                {
                    Logger.err("cut width is too small");
                    base.MachineOpStatus = MachineOpStatus.Errors;
                    return;
                }

                if (_stepover.Cached == 0 || _stepover.Cached > 1)
                {
                    Logger.err("stepover should be > 0 and <= 1");
                    base.MachineOpStatus = MachineOpStatus.Errors;
                    return;
                }

                if (_should_overcut_corners.Cached && _should_clear_inside_corners)
                {
                    Logger.err("Corner Overcut and Clear Inside Corners can't be used together");
                    base.MachineOpStatus = MachineOpStatus.Errors;
                    return;
                }

                // XXX: is it needed ?
                base.UpdateGeometryExtrema(base._CADFile);
                base._CADFile.MachiningOptions.UpdateGeometryExtrema(base._CADFile);
                ShapeList shapes = new ShapeList();
                shapes.ApplyTransformations = true;
                shapes.AddEntities(base._CADFile, base.PrimitiveIds);
                shapes = shapes.DetectRegions();

                List<Sliced_path> trajectories = new List<Sliced_path>();

                bool is_inside = _cut_side.Cached == InsideOutsideOptions.Inside;

                Point2F startpoint = (Point2F)base.StartPoint.Cached;

                foreach (ShapeListItem shape in shapes)
                {
                    if (trajectories.Count != 0)
                        startpoint = lastpt(trajectories);

                    if (shape.Shape is Polyline)
                        trajectories.AddRange(gen_profile((Polyline)shape.Shape, is_inside, startpoint));
                    else if (shape.Shape is CamBam.CAD.Region)
                        trajectories.AddRange(gen_profile((CamBam.CAD.Region)shape.Shape, is_inside, startpoint));
                }

                if (trajectories.Count == 0)
                    return;

                base.insert_toolpaths(trajectories);

                if (base.MachineOpStatus == MachineOpStatus.Unknown)
                {
                    base.MachineOpStatus = MachineOpStatus.OK;
                }
            }
            catch (Exception ex)
            {
                base.MachineOpStatus = MachineOpStatus.Errors;
                ThisApplication.HandleException(ex);
            }
            finally
            {
                base._GenerateToolpathsFinal();
            }
        }

        public override MachineOp Clone()
        {
            return new MOPTrochoprof(this);
        }

        public MOPTrochoprof(MOPTrochoprof src) : base(src)
        {
            this.InsideOutside = src.InsideOutside;
            this.CutWidth = src.CutWidth;
            this.CornerOvercut = src.CornerOvercut;
            this.Clear_inside_corners = src.Clear_inside_corners;
            this.Min_stepover = src.Min_stepover;
        }

        public MOPTrochoprof()
        {
        }

        public MOPTrochoprof(CADFile CADFile, ICollection<Entity> plist) : base(CADFile, plist)
        {
        }
    }

}
