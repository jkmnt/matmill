using System;
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
    [Serializable]
    public class Mop_matmill : MOPFromGeometry, IIcon
    {
        List<List<Pocket_path_item>> _pockets = new List<List<Pocket_path_item>>();
        List<Surface> _cut_widths = new List<Surface>();

        protected CBValue<double> _stepover;
        protected CBValue<double> _min_stepover_percentage;
        protected CBValue<double> _chord_feedrate;
        protected CBValue<double> _depth_increment;
        protected CBValue<double> _final_depth_increment;
        protected CBValue<double> _target_depth;
        protected CBValue<CutOrderingOption> _cut_ordering;
        protected CBValue<MillingDirectionOptions> _milling_direction;

        [Browsable(false), XmlIgnore]
		public override string MOPTypeName
		{
			get { return "TrochoPock"; }
		}

        [Browsable(false)]
        public Image ActiveIconImage
        {
        	get { return resources.cam_trochopock1;}
        }

        [Browsable(false)]
        public string ActiveIconKey
        {
        	get { return "cam_trochopock1"; }
        }

        [Browsable(false)]
        public Image InactiveIconImage
        {
        	get { return resources.cam_trochopock0;}
        }

        [Browsable(false)]
        public string InactiveIconKey
        {
        	get { return "cam_trochopock0"; }
        }

        // hide some non-relevant base parameters

        [Browsable(false), XmlIgnore]
        public new CBValue<OptimisationModes> OptimisationMode
        {
            get { return base.OptimisationMode; }
            set { }
        }

        [CBKeyValue, Category("Feedrates"), DefaultValue(2000.0), Description("Feedrate for the chords and movements inside the milled pocket"), DisplayName("Chord Feedrate")]
		public CBValue<double> Chord_feedrate
		{
			get	{ return _chord_feedrate; }
			set {
                _chord_feedrate = value;

                if (_chord_feedrate.IsDefault)
                {
                    _chord_feedrate.SetCache(2000.0);
                    _chord_feedrate.Value = _chord_feedrate.Cached;
                }
            }
		}

        [CBKeyValue, Category("Step Over"),
            DefaultValue(typeof(CBValue<double>), "Default"),
            Description("The cut is increased by this amount each step, expressed as a decimal (0-1.0) of the cutter width."),
            DisplayName("Step Over")]
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
        	get
        	{
        		return this._cut_ordering;
        	}
        	set
        	{
        		this._cut_ordering = value;
        	}
        }


        [CBKeyValue, Category("Cutting Depth"), DefaultValue(typeof(CBValue<double>), "Default"), Description("Final depth of the machining operation."), DisplayName("Target Depth")]
        public CBValue<double> TargetDepth
        {
        	get { return this._target_depth; }
        	set { this._target_depth = value; }
        }

        [CBAdvancedValue, Category("Step Over"), DefaultValue(0.9), Description("Minimum allowed stepover as a percentage of the nominal stepover (0.1 - 0.9)"), DisplayName("Minimum Stepover")]
		public CBValue<double> Min_stepover
		{
			get { return this._min_stepover_percentage; }
			set
            {
                if (value.IsValue && (value.Value < 0.1 || value.Value > 0.9))
                {
                    _min_stepover_percentage.SetState(CBValueStates.Error);
                    Host.err("Min Stepover must be in range 0.1 - 0.9");
                    return;
                }

                _min_stepover_percentage = value;

                if (_min_stepover_percentage.IsDefault)
                {
                    _min_stepover_percentage.SetCache(0.9);
                    _min_stepover_percentage.Value = _min_stepover_percentage.Cached;
                }
            }
		}

        private double[] get_z_layers()
        {
            return base.GetZLayers(base.StockSurface.Cached, this.TargetDepth.Cached, this.DepthIncrement.Cached, this.FinalDepthIncrement.Cached);
        }

        private List<Pocket_path_item> gen_pocket(ShapeListItem shape)
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
            gen.Cutter_d = base.ToolDiameter.Cached;
            gen.Max_engagement = base.ToolDiameter.Cached * _stepover.Cached;
            gen.Min_engagement = base.ToolDiameter.Cached * _stepover.Cached * _min_stepover_percentage.Cached;

            Pocket_path_item_type to_emit = Pocket_path_item_type.LEADIN_SPIRAL | Pocket_path_item_type.BRANCH_ENTRY | Pocket_path_item_type.SEGMENT;
            if (get_z_layers().Length > 1)
                to_emit |= Pocket_path_item_type.RETURN_TO_BASE;
            gen.Emit_options = to_emit;

            gen.Startpoint = (Point2F)base.StartPoint.Cached;
            gen.Margin = base.RoughingClearance.Cached;

            int spindle_dir = (int)(base.SpindleDirection.Cached != SpindleDirectionOptions.Off ? SpindleDirection.Cached : SpindleDirectionOptions.CW);
            gen.Mill_direction = (RotationDirection)(_milling_direction.Cached == MillingDirectionOptions.Conventional ?  spindle_dir : -spindle_dir);

            return gen.run();
        }

		protected override void _GenerateToolpathsWorker()
		{
			try
			{
                _pockets.Clear();
                _cut_widths.Clear();
				GC.Collect();

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
                    Host.warn("Found open polylines: ignoring");
                    this.MachineOpStatus = MachineOpStatus.Warnings;
                }

                foreach (ShapeListItem shape in shapes)
                {
                    List<Pocket_path_item> pocket = gen_pocket(shape);
                    if (pocket != null)
                        _pockets.Add(pocket);
                }

                double[] depths = get_z_layers();
                update_cut_widths(depths[depths.Length - 1]);

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

        private void emit_pocket_at_depth(MachineOpToGCode gcg, List<Pocket_path_item> pocket, double depth)
        {
            // first item is the spiral lead-in by convention
            if (pocket[0].Item_type != Pocket_path_item_type.LEADIN_SPIRAL)
                throw new Exception("no spiral lead-in in pocket path");


            foreach (Pocket_path_item item in pocket)
            {
                switch (item.Item_type)
                {
                    case Pocket_path_item_type.LEADIN_SPIRAL:
                        // rapid/plunge should be inserted by gcg automatically if required
                        gcg.AppendPolyLine(pocket[0], depth);
                        break;

                    // emit chordal segment to the start of polyline with the chord feedrate.
                    // looks like this low-level hack is the only way to include move with custom feedrate it cambam
                    // no retracts/rapids are needed, path is continuous
                    // emit arc after that. gcg should change feedrate to the cut feedrate internally
                    case Pocket_path_item_type.SEGMENT:
                        {
                            Point3F pt = new Point3F(item.Points[0].Point.X, item.Points[0].Point.Y, depth);
                            gcg.ApplyGCodeOrigin(ref pt);
                            gcg.AppendMove("g1", pt.X, pt.Y, pt.Z, _chord_feedrate.Cached);
                            gcg.AppendPolyLine(item, depth);
                        }
                        break;

                    case Pocket_path_item_type.BRANCH_ENTRY:
                    case Pocket_path_item_type.RETURN_TO_BASE:
                        {
                            // emit move inside the pocket point-by-point to preserve custom feedrate, same hack
                            // no retracts/rapids are needed, path is continuous
                            for (int i = 0; i < item.Points.Count; i++)
                            {
                                Point3F pt = new Point3F(item.Points[i].Point.X, item.Points[i].Point.Y, depth);
                                gcg.ApplyGCodeOrigin(ref pt);
                                gcg.AppendMove("g1", pt.X, pt.Y, pt.Z, _chord_feedrate.Cached);
                            }
                        }
                        break;

                    default:
                        throw new Exception("unknown item type in pocket path");
                }
            }
        }

        public override void PostProcess(MachineOpToGCode gcg)
        {
        	gcg.DefaultStockHeight = base.StockSurface.Cached;

            if (_pockets.Count == 0) return;

            double[] depths = get_z_layers();

            if (_cut_ordering.Cached == CutOrderingOption.DepthFirst)
            {
                foreach (List<Pocket_path_item> pocket in _pockets)
                {
                    foreach (double depth in depths)
                        emit_pocket_at_depth(gcg, pocket, depth);
                }
            }
            else
            {
                foreach (double depth in depths)
                {
                    foreach (List<Pocket_path_item> pocket in _pockets)
                        emit_pocket_at_depth(gcg, pocket, depth);
                }
            }
        }

        private void paint_pocket(ICADView iv, Display3D d3d, Color arccolor, Color linecolor, List<Pocket_path_item> pocket, double[] depths)
        {
            for (int didx = depths.Length - 1; didx >= 0; didx--)
            {
                double depth = depths[didx];

                foreach (Pocket_path_item item in pocket)
                {
                    Matrix4x4F matrix4x4F = new Matrix4x4F();
                    matrix4x4F.Translate(0.0, 0.0, depth);
                    if (base.Transform.Cached != null)
                        matrix4x4F *= base.Transform.Cached;

                    d3d.ModelTransform = matrix4x4F;
                    d3d.LineWidth = 1F;
                    item.Paint(d3d, arccolor, linecolor);
                    base.PaintDirectionVector(iv, item, d3d, matrix4x4F);
                }
            }
        }

        private void update_cut_widths(double depth)
        {
            _cut_widths.Clear();

            foreach (List<Pocket_path_item> pocket in _pockets)
            {
                foreach(Pocket_path_item item in pocket)
                {
                    Polyline p = item;

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
    				_cut_widths.Add(surface);
                }
            }
		}


        public override void Paint(ICADView iv, Display3D d3d, Color arccolor, Color linecolor, bool selected)
        {
        	this._CADFile = iv.CADFile;

            if (_pockets.Count == 0) return;

            double[] depths = get_z_layers();

            foreach (List<Pocket_path_item> pocket in _pockets)
            {
                paint_pocket(iv, d3d, arccolor, linecolor, pocket, depths);
            }

            if (this._CADFile.ShowCutWidths)
            {
                d3d.LineColor = CamBamConfig.Defaults.CutWidthColor;
                d3d.LineStyle = LineStyle.Solid;
                d3d.LineWidth = 0F;
                d3d.ModelTransform = Matrix4x4F.Identity;
                d3d.UseLighting = false;
                foreach (Surface s in _cut_widths)
                {
                    s.Paint(d3d);
                }
                d3d.UseLighting = true;
            }

        	if (selected)
        	{
        		base.PaintStartPoint(iv, d3d);
        	}

//                if (num3 < 0 && num4 < 0 && this._CADFile.ShowRapids)
//                {
//                    d3d.LineWidth = 1f;
//                    d3d.LineColor = CamBamConfig.Defaults.ToolpathRapidColor;
//                    d3d.ModelTransform = Matrix4x4F.Identity;
//                    d3d.LineStyle = LineStyle.Dotted;
//                    foreach (Polyline current2 in base.Toolpaths2.Rapids)
//                    {
//                        current2.Paint(d3d);
//                    }
//                    d3d.LineStyle = LineStyle.Solid;
//                }
//                if (this._CADFile.ShowCutWidths && base.Toolpaths2.Sequence.Count > 0 && base.Toolpaths2.CutWidths != null)
//                {
//                    d3d.LineColor = CamBamConfig.Defaults.CutWidthColor;
//                    d3d.LineStyle = LineStyle.Solid;
//                    d3d.LineWidth = 0f;
//                    d3d.ModelTransform = Matrix4x4F.Identity;
//                    d3d.UseLighting = false;
//                    foreach (Surface current3 in base.Toolpaths2.CutWidths)
//                    {
//                        current3.Paint(d3d);
//                    }
//                    d3d.UseLighting = true;
//                }
        }

        public override MachineOp Clone()
        {
            return new Mop_matmill(this);
        }

        public Mop_matmill(Mop_matmill src) : base(src)
        {
            StepOver = src.StepOver;
            Min_stepover = src.Min_stepover;
            Chord_feedrate = src.Chord_feedrate;
            DepthIncrement = src.DepthIncrement;
            FinalDepthIncrement = src.FinalDepthIncrement;
            TargetDepth = src.TargetDepth;
            MillingDirection = src.MillingDirection;
        }

        public Mop_matmill()
        {

        }

        public Mop_matmill(CADFile CADFile, ICollection<Entity> plist) : base(CADFile, plist)
        {
        }

    }

}