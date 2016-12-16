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

namespace Trochopock
{
    [Serializable]
    public class MOPTrochoprof : Sliced_mop, IIcon
    {
        protected CBValue<InsideOutsideOptions> _cut_side;
        protected CBValue<double> _cut_width;
        protected CBValue<bool> _should_overcut_corners;

        //--- invisible and non-serializable properties

        [XmlIgnore, Browsable(false)]
        public override string MOPTypeName
        {
            get { return "TrochoProf"; }
        }

        [XmlIgnore, Browsable(false)]
        public Image ActiveIconImage
        {
            get { return resources.cam_trochopock1;}
        }

        [XmlIgnore, Browsable(false)]
        public string ActiveIconKey
        {
            get { return "cam_trochopock1"; }
        }

        [XmlIgnore, Browsable(false)]
        public Image InactiveIconImage
        {
            get { return resources.cam_trochopock0;}
        }

        [XmlIgnore, Browsable(false)]
        public string InactiveIconKey
        {
            get { return "cam_trochopock0"; }
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

        private Sliced_path gen_toolpath(Polyline poly, double width, Point2F startpoint)
        {
            Engrave_generator gen = new Engrave_generator(poly);

            gen.General_tolerance = is_inch_units() ? 0.001 / 25.4 : 0.001;
            gen.Tool_d = base.ToolDiameter.Cached;
            gen.Max_ted = base.ToolDiameter.Cached * _stepover.Cached;
            gen.Startpoint = startpoint;

            gen.Slice_radius = (width - base.ToolDiameter.Cached) / 2;

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

        private List<Sliced_path> gen_profile(Polyline poly, bool is_inside, Point2F startpoint)
        {
            List<Sliced_path> trajectories = new List<Sliced_path>();

            double cut_width = _cut_width.Cached;
            if (cut_width == 0)
                cut_width = base.ToolDiameter.Cached * 2;

            double offset = cut_width / 2 + base.RoughingClearance.Cached;
            if (is_inside)
                offset = -offset;

            Polyline[] array = poly.CreateOffsetPolyline(offset, (double)CamBamConfig.Defaults.GeneralTolerance, _should_overcut_corners.Cached, false);

            if (array == null)
                return trajectories;

            foreach (Polyline p in array)
            {
                if (trajectories.Count != 0)
                    startpoint = lastpt(trajectories);

                Sliced_path toolpath = gen_toolpath(p, cut_width, startpoint);
                if (toolpath != null)                
                    trajectories.Add(toolpath);
            }

            return trajectories;
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
                    Host.err("tool diameter is zero");
                    base.MachineOpStatus = MachineOpStatus.Errors;
                    return;
                }

                if (_cut_width.Cached != 0 && _cut_width.Cached < base.ToolDiameter.Cached * 1.05)
                {
                    Host.err("cut width is too small");
                    base.MachineOpStatus = MachineOpStatus.Errors;
                    return;
                }

                if (_stepover.Cached == 0 || _stepover.Cached > 1)
                {
                    Host.err("stepover should be > 0 and <= 1");
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
        }

        public MOPTrochoprof()
        {
        }

        public MOPTrochoprof(CADFile CADFile, ICollection<Entity> plist) : base(CADFile, plist)
        {
        }
    }

}
