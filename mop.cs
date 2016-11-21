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

namespace Matmill
{
    // toolpath combines flat trajectory and depth
    class Toolpath
    {
        public readonly Pocket_path Trajectory;
        public readonly double Depth;
        public readonly bool Should_return_to_base;
        public Toolpath(Pocket_path path, double depth, bool return_to_base)
        {
            this.Trajectory = path;
            this.Depth = depth;
            this.Should_return_to_base = return_to_base;
        }
    }

    [Serializable]
    public class Mop_matmill : MOPFromGeometry, IIcon
    {
        [NonSerialized]
        private List<Pocket_path> _trajectories = new List<Pocket_path>();
        [NonSerialized]
        private List<Toolpath> _toolpaths = new List<Toolpath>();

        //--- these are for rendering only !
        [NonSerialized]
        private List<Surface> _visual_cut_widths = new List<Surface>();
        [NonSerialized]
        private List<Polyline> _visual_rapids = new List<Polyline>();

        //--- mop properties

        protected CBValue<CutOrderingOption> _cut_ordering;
        protected CBValue<MillingDirectionOptions> _milling_direction;
        protected CBValue<double> _depth_increment;
        protected CBValue<double> _final_depth_increment;
        protected CBValue<double> _stepover;
        protected CBValue<double> _target_depth;

        protected double _chord_feedrate = 2000.0;
        protected double _min_stepover_percentage = 0.9;
        protected double _segmented_slice_derating = 0.5;
        protected bool _may_return_to_base = true;

        //--- invisible and non-serializeable properties

        [XmlIgnore, Browsable(false)]
		public override string MOPTypeName
		{
			get { return "TrochoPock"; }
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

        //--- hidden base parameters non-relevant to this mop

        [XmlIgnore, Browsable(false)]
        public new CBValue<OptimisationModes> OptimisationMode
        {
            get { return base.OptimisationMode; }
            set { }
        }

        //--- visible parameters which may be styled

        [CBKeyValue, Category("Step Over"), DefaultValue(typeof(CBValue<double>), "Default"), Description("The cut is increased by this amount each step, expressed as a decimal (0-1.0) of the cutter width."), DisplayName("Step Over")]
        public CBValue<double> StepOver
        {
        	get { return this._stepover; }
        	set { this._stepover = value; }
        }

        [CBKeyValue, Category("Cutting Depth"), DefaultValue(typeof(CBValue<double>), "Default"), Description("Depth increment of each machining pass."), DisplayName("Depth Increment")]
        public CBValue<double> DepthIncrement
        {
        	get	{ return this._depth_increment; }
        	set	{ this._depth_increment = value; }
        }

        [Category("Cutting Depth"), DefaultValue(typeof(CBValue<double>), "Default"), Description("The depth increment of the final machining pass."), DisplayName("Final Depth Increment")]
        public CBValue<double> FinalDepthIncrement
        {
        	get { return this._final_depth_increment; }
        	set { this._final_depth_increment = value; }
        }

        [Category("Options"), DefaultValue(typeof(CBValue<MillingDirectionOptions>), "Default"), Description("Controls the direction the cutter moves around the toolpath.\r\nConventional or Climb milling supported."), DisplayName("Milling Direction")]
        public CBValue<MillingDirectionOptions> MillingDirection
        {
        	get { return this._milling_direction; }
        	set { this._milling_direction = value; }

        }

        [Category("Options"), DefaultValue(typeof(CBValue<CutOrderingOption>), "Default"), Description("Controls whether to cut to depth first or all cuts on this level first."), DisplayName("Cut Ordering")]
        public CBValue<CutOrderingOption> CutOrdering
        {
        	get { return this._cut_ordering; }
        	set { this._cut_ordering = value; }
        }


        [CBKeyValue, Category("Cutting Depth"), DefaultValue(typeof(CBValue<double>), "Default"), Description("Final depth of the machining operation."), DisplayName("Target Depth")]
        public CBValue<double> TargetDepth
        {
        	get { return this._target_depth; }
        	set { this._target_depth = value; }
        }

        //--- our own new parameters. No reason to make them CBValues, since they couldn't be styled anyway

        [
            CBKeyValue,
            Category("Feedrates"),
            DefaultValue(2000),
            Description("Feedrate for the chords and movements inside the milled pocket"),
            DisplayName("Chord Feedrate")
        ]
        public double Chord_feedrate
        {
            get	{ return _chord_feedrate; }
            set { _chord_feedrate = value; }
        }

        [
            CBAdvancedValue,
            Category("Step Over"),
            DefaultValue(0.9),
            Description("Minimum allowed stepover as a percentage of the nominal stepover (0.1 - 0.9).\nLarger values may leave uncut corners"),
            DisplayName("Minimum Stepover")
        ]
		public double Min_stepover
		{
			get { return this._min_stepover_percentage; }
			set
            {
                _min_stepover_percentage = value;

                if (value < 0.1 || value > 0.9)
                {
                    _min_stepover_percentage = Math.Max(Math.Min(0.9, value), 0.1);
                    redraw_parameters();
                }
            }
		}

        [
            CBAdvancedValue,
            Category("Step Over"),
            DefaultValue(0.5),
            Description("Derating factor for the segmented slices to reduce cutter stress (0 - 1)"),
            DisplayName("Segmented Slices Derating")
        ]
		public double Segmented_slice_derating
		{
			get { return this._segmented_slice_derating; }
			set { _segmented_slice_derating = value; }
		}

        [
            CBAdvancedValue,
            Category("Options"),
            Description("Try to return to the start point without rapids (if required)"),
            DisplayName("Return To Base")
        ]
		public bool May_return_to_base
		{
			get { return this._may_return_to_base; }
			set { _may_return_to_base = value; }
		}

        //-- read-only About field

        [
            XmlIgnore,
            Category("Misc"),
            DisplayName("Plugin Version"),
            Description("https://github.com/jkmnt/matmill\njkmnt at git@firewood.fastmail.com")
        ]
		public string Version
		{
            get { return Assembly.GetExecutingAssembly().GetName().Version.ToString(); }
		}

        public override bool NeedsRebuild
        {
            get { return _trajectories == null || _trajectories.Count == 0; }
        }

        private double[] get_z_layers()
        {
            return base.GetZLayers(base.StockSurface.Cached, this.TargetDepth.Cached, this.DepthIncrement.Cached, this.FinalDepthIncrement.Cached);
        }

        private List<Toolpath> gen_ordered_toolpath(List<Pocket_path> trajectories, double[] depths)
        {
            List<Toolpath> toolpaths = new List<Toolpath>();

            // cut ordering == level first is meaningful only for a several pockets
            if (trajectories.Count < 2 || _cut_ordering.Cached == CutOrderingOption.DepthFirst)
            {
                foreach (Pocket_path traj in trajectories)
                {
                    // last depth level of each pocket should have no return to base, it's useless
                    // intermediate levels may have returns if not disabled by setting
                    int i;
                    for (i = 0; i < depths.Length - 1; i++)
                        toolpaths.Add(new Toolpath(traj, depths[i], _may_return_to_base));
                    toolpaths.Add(new Toolpath(traj, depths[i], false));
                }
            }
            else
            {
                foreach (double depth in depths)
                {
                    foreach (Pocket_path traj in trajectories)
                        toolpaths.Add(new Toolpath(traj, depth, false));
                }
            }

            return toolpaths;
        }

        private Pocket_path gen_pocket(ShapeListItem shape)
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

            gen.General_tolerance = _CADFile.DrawingUnits == Units.Inches ? 0.001 / 25.4 : 0.001;
            gen.Cutter_d = base.ToolDiameter.Cached;
            gen.Max_engagement = base.ToolDiameter.Cached * _stepover.Cached;
            gen.Min_engagement = base.ToolDiameter.Cached * _stepover.Cached * _min_stepover_percentage;
            gen.Segmented_slice_engagement_derating_k = _segmented_slice_derating;

            Pocket_path_item_type to_emit =     Pocket_path_item_type.LEADIN_SPIRAL
                                              | Pocket_path_item_type.BRANCH_ENTRY
                                              | Pocket_path_item_type.SEGMENT
                                              | Pocket_path_item_type.RETURN_TO_BASE;
            gen.Emit_options = to_emit;

            gen.Startpoint = (Point2F)base.StartPoint.Cached;
            gen.Margin = base.RoughingClearance.Cached;

            if (_milling_direction.Cached == MillingDirectionOptions.Mixed || base.SpindleDirection.Cached == SpindleDirectionOptions.Off)
            {
                gen.Mill_direction = RotationDirection.Unknown; // means 'mixed' here
            }
            else
            {
                int dir = (int)(base.SpindleDirection.Cached);
                if (_milling_direction.Cached == MillingDirectionOptions.Climb)
                    dir = -dir;
                gen.Mill_direction = (RotationDirection)dir;
            }

            return gen.run();
        }

        private void redraw_parameters()
        {
            CamBamUI.MainUI.ObjectProperties.Refresh();
        }

        private List<Surface> calc_visual_cut_widths(List<Pocket_path> trajectories, double depth)
        {
            List<Surface> surfaces = new List<Surface>();

            foreach (Pocket_path traj in trajectories)
            {
                foreach(Pocket_path_item item in traj)
                {
                    Polyline p = item;

                    if (item.Item_type != Pocket_path_item_type.SEGMENT && item.Item_type != Pocket_path_item_type.LEADIN_SPIRAL)
                        continue;

                    // TODO: maybe a single transform of surface will suffice ?
                    if (base.Transform.Cached != null && ! base.Transform.Cached.IsIdentity())
                    {
                        p = (Polyline) p.Clone();
                        p.ApplyTransformation(base.Transform.Cached);
                    }

                    PolylineToMesh mesh = new PolylineToMesh(p);
    				Matrix4x4F xm = Matrix4x4F.Translation(0.0, 0.0, depth - 0.001);
    				Surface surface = mesh.ToWideLine(base.ToolDiameter.Cached);
    				surface.ApplyTransformation(xm);
    				surfaces.Add(surface);
                }
            }

            return surfaces;
		}

        private List<Polyline> calc_visual_rapids(List<Toolpath> toolpaths)
        {
            List<Polyline> rapids = new List<Polyline>();

            double thres = base.GetDistanceThreshold();

            Point3F lastpt = Point3F.Undefined;

            // rapids are possible only between depth levels of pocket and separate pockets
            foreach (Toolpath path in toolpaths)
            {
                if (! lastpt.IsUndefined)
                {
                    Point3F to = path.Trajectory[0].FirstPoint;
                    to = new Point3F(to.X, to.Y, path.Depth);

                    double dist = Point2F.Distance((Point2F)lastpt, (Point2F)to);

                    if (dist > thres + (double)CamBamConfig.Defaults.GeneralTolerance)
                    {
                        // rapid here from last to first point of pocket
                        Polyline p = new Polyline();
                        p.Add(lastpt);
                        p.Add(new Point3F(lastpt.X, lastpt.Y, ClearancePlane.Cached));
                        p.Add(new Point3F(to.X, to.Y, ClearancePlane.Cached));
                        p.Add(to);
                        rapids.Add(p);
                    }
                }

                // NOTE: we're discarding last path item if return to base should be disabled for this segment
                lastpt = path.Trajectory[path.Trajectory.Count - (path.Should_return_to_base ? 1 : 2)].LastPoint;
                lastpt = new Point3F(lastpt.X, lastpt.Y, path.Depth);
            }

            return rapids;
		}

		protected override void _GenerateToolpathsWorker()
		{
			try
			{
                _trajectories.Clear();
                _toolpaths.Clear();
                _visual_cut_widths.Clear();
                _visual_rapids.Clear();
				GC.Collect();

                if (base.ToolDiameter.Cached == 0)
                {
                    Host.err("tool diameter is zero");
                    this.MachineOpStatus = MachineOpStatus.Errors;
                    return;
                }

                if (_stepover.Cached == 0 || _stepover.Cached > 1)
                {
                    Host.err("stepover should be > 0 and <= 1");
                    this.MachineOpStatus = MachineOpStatus.Errors;
                    return;
                }

                // XXX: is it needed ?
				base.UpdateGeometryExtrema(this._CADFile);
				this._CADFile.MachiningOptions.UpdateGeometryExtrema(this._CADFile);
                ShapeList shapes = new ShapeList();
                shapes.ApplyTransformations = true;
                shapes.AddEntities(this._CADFile, base.PrimitiveIds);
                shapes = shapes.DetectRegions();

                bool found_opened_polylines = false;
                for (int i = shapes.Count - 1; i >= 0; i--)
                {
                    if (shapes[i].Shape is Polyline && ! ((Polyline)shapes[i].Shape).Closed)
                    {
                        found_opened_polylines = true;
                        shapes.RemoveAt(i);
                    }
                }
                if (found_opened_polylines)
                {
                    Host.warn("ignoring open polylines");
                    this.MachineOpStatus = MachineOpStatus.Warnings;
                }

                foreach (ShapeListItem shape in shapes)
                {
                    Pocket_path traj = gen_pocket(shape);
                    if (traj != null)
                        _trajectories.Add(traj);
                }

                if (_trajectories.Count == 0)
                    return;

                double[] depths = get_z_layers();
                _toolpaths = gen_ordered_toolpath(_trajectories, depths);
                _visual_cut_widths = calc_visual_cut_widths(_trajectories, depths[depths.Length - 1]);  // for the last depth only
                _visual_rapids = calc_visual_rapids(_toolpaths);

                if (this.MachineOpStatus == MachineOpStatus.Unknown)
                {
                    this.MachineOpStatus = MachineOpStatus.OK;
                }
			}
			catch (Exception ex)
			{
				this.MachineOpStatus = MachineOpStatus.Errors;
				ThisApplication.HandleException(ex);
			}
			finally
			{
				this._GenerateToolpathsFinal();
			}
		}

        private void emit_toolpath(MachineOpToGCode gcg, Toolpath path)
        {
            // first item is the spiral lead-in by convention
            if (path.Trajectory[0].Item_type != Pocket_path_item_type.LEADIN_SPIRAL)
                throw new Exception("no spiral lead-in in pocket path");

            // make sure there would be rapid if required
            gcg.CheckPosition(path.Trajectory[0].Points[0].Point, path.Depth);

            foreach (Pocket_path_item item in path.Trajectory)
            {
                if (item.Item_type == Pocket_path_item_type.LEADIN_SPIRAL)
                {
                    gcg.AppendPolyLine(item, path.Depth);
                }
                else if (item.Item_type == Pocket_path_item_type.SEGMENT)
                {
                    // emit chordal segment to the start of polyline with the chord feedrate.
                    // looks like this low-level hack is the only way to include move with custom feedrate it cambam
                    // no retracts/rapids are needed, path is continuous
                    // emit arc after that. gcg should change feedrate to the cut feedrate internally
                    Point3F pt = new Point3F(item.Points[0].Point.X, item.Points[0].Point.Y, path.Depth);
                    gcg.ApplyGCodeOrigin(ref pt);
                    gcg.AppendMove("g1", pt.X, pt.Y, pt.Z, _chord_feedrate);
                    gcg.AppendPolyLine(item, path.Depth);
                }
                else if (item.Item_type == Pocket_path_item_type.BRANCH_ENTRY || item.Item_type == Pocket_path_item_type.RETURN_TO_BASE)
                {
                    if (item.Item_type == Pocket_path_item_type.RETURN_TO_BASE && !path.Should_return_to_base)
                        continue;

                    // emit move inside the pocket point-by-point to preserve custom feedrate, same hack
                    // no retracts/rapids are needed, path is continuous
                    for (int i = 0; i < item.Points.Count; i++)
                    {
                        Point3F pt = new Point3F(item.Points[i].Point.X, item.Points[i].Point.Y, path.Depth);
                        gcg.ApplyGCodeOrigin(ref pt);
                        gcg.AppendMove("g1", pt.X, pt.Y, pt.Z, _chord_feedrate);
                    }
                }
                else
                {
                    throw new Exception("unknown item type in pocket trajectory");
                }
            }
        }

        private void paint_pocket(ICADView iv, Display3D d3d, Color arccolor, Color linecolor, Toolpath path)
        {
            foreach (Pocket_path_item p in path.Trajectory)
            {
                if (p.Item_type == Pocket_path_item_type.RETURN_TO_BASE && (!path.Should_return_to_base))
                    continue;

                Matrix4x4F matrix4x4F = new Matrix4x4F();
                matrix4x4F.Translate(0.0, 0.0, path.Depth);
                if (base.Transform.Cached != null)
                    matrix4x4F *= base.Transform.Cached;

                d3d.ModelTransform = matrix4x4F;
                d3d.LineWidth = 1F;
                p.Paint(d3d, arccolor, linecolor);
                base.PaintDirectionVector(iv, p, d3d, matrix4x4F);
            }
        }

        public override void PostProcess(MachineOpToGCode gcg)
        {
            gcg.DefaultStockHeight = base.StockSurface.Cached;

            if (_trajectories.Count == 0)
                return;

            foreach(Toolpath path in _toolpaths)
                emit_toolpath(gcg, path);
        }

        private void paint_cut_widths(Display3D d3d)
        {
            double depth = _toolpaths[_toolpaths.Count - 1].Depth;

            d3d.LineColor = CamBamConfig.Defaults.CutWidthColor;
            d3d.LineStyle = LineStyle.Solid;
            d3d.LineWidth = 0F;
            d3d.ModelTransform = Matrix4x4F.Identity;
            d3d.UseLighting = false;
            foreach (Surface s in _visual_cut_widths)
                s.Paint(d3d);

            d3d.UseLighting = true;
        }

        private void paint_rapids(Display3D d3d)
        {
            d3d.LineWidth = 1f;
            d3d.LineColor = CamBamConfig.Defaults.ToolpathRapidColor;
            d3d.ModelTransform = Matrix4x4F.Identity;
            d3d.LineStyle = LineStyle.Dotted;

            foreach (Polyline p in _visual_rapids)
                p.Paint(d3d);

            d3d.LineStyle = LineStyle.Solid;
        }

        public override void Paint(ICADView iv, Display3D d3d, Color arccolor, Color linecolor, bool selected)
        {
            // XXX: what this line for ?
        	this._CADFile = iv.CADFile;

            if (_trajectories.Count == 0) return;

            foreach (Toolpath item in _toolpaths)
                paint_pocket(iv, d3d, arccolor, linecolor, item);

            if (this._CADFile.ShowCutWidths)
                paint_cut_widths(d3d);

            if (this._CADFile.ShowRapids)
                paint_rapids(d3d);

            if (selected)
        		base.PaintStartPoint(iv, d3d);
        }

        public override MachineOp Clone()
        {
            return new Mop_matmill(this);
        }

        public Mop_matmill(Mop_matmill src) : base(src)
        {
            CutOrdering = src.CutOrdering;
            MillingDirection = src.MillingDirection;
            DepthIncrement = src.DepthIncrement;
            FinalDepthIncrement = src.FinalDepthIncrement;
            StepOver = src.StepOver;
            TargetDepth = src.TargetDepth;

            Chord_feedrate = src.Chord_feedrate;
            Min_stepover = src.Min_stepover;
            Segmented_slice_derating = src.Segmented_slice_derating;
        }

        public Mop_matmill()
        {


        }

        public Mop_matmill(CADFile CADFile, ICollection<Entity> plist) : base(CADFile, plist)
        {
        }

    }

}