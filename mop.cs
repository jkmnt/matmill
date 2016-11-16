using CamBam;
using CamBam.CAD;
using CamBam.CAM;
using CamBam.Geom;
using CamBam.Library;
using CamBam.UI;
using CamBam.Util;
using CamBam.Values;
using System;
using System.Collections;
using System.Collections.Generic;
using System.ComponentModel;
using System.Drawing;
using System.Xml.Serialization;

namespace Matmill
{
    [Serializable]
    public class Mop_matmill : MOPFromGeometry
    {
        List<Entity> _path = new List<Entity>();

        protected CBValue<double> _stepover;
        protected CBValue<double> _min_stepover_percentage;
        protected CBValue<double> _chord_feedrate;
        protected CBValue<double> _depth_increment;
        protected CBValue<double> _final_depth_increment;
        protected CBValue<double> _target_depth;
        protected CBValue<MillingDirectionOptions> _milling_direction;

        [Browsable(false), XmlIgnore]
		public override string MOPTypeName
		{
			get
			{
				return "TrochoPock";
			}
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

		protected override void _GenerateToolpathsWorker()
		{
			try
			{
                _path.Clear();
				GC.Collect();

                // XXX: is it needed ?
				base.UpdateGeometryExtrema(this._CADFile);
				this._CADFile.MachiningOptions.UpdateGeometryExtrema(this._CADFile);

                List<Polyline> polys = new List<Polyline>();

                foreach (int id in base.PrimitiveIds)
                {
                    Entity e = this._CADFile.FindPrimitive(id);

                    if (e == null)
                        continue;

                    if (e is Polyline)
                        polys.Add((Polyline)e);
                    if (e is Circle)
                        polys.Add(((Circle)e).ToPolyline());
                    else if (e is MText)
                        polys.AddRange(((MText)e).ConvertToPolylines(true));
                    else if (e is CamBam.CAD.Region)
                        polys.AddRange(((CamBam.CAD.Region)e).ConvertToPolylines(true));
                }

                bool has_opened_polylines = false;
				for (int i = polys.Count - 1; i >= 0; i--)
				{
					Polyline p = polys[i];
					if (! p.Closed)
					{
						has_opened_polylines = true;
						polys.RemoveAt(i);
					}
				}
				if (has_opened_polylines)
				{
                    Host.warn("Found open polylines: ignoring");
					this.MachineOpStatus = MachineOpStatus.Warnings;
				}

                if (polys.Count == 0) return;

                CamBam.CAD.Region[] regs = CamBam.CAD.Region.CreateFromPolylines(polys.ToArray());

                if (regs.Length == 0) return;

                if (regs.Length > 1)
                {
                    Host.warn("Found several pockets to mill: ignoring all except the first one");
                }

                CamBam.CAD.Region reg = regs[0];

                Pocket_generator gen = new Pocket_generator(reg.OuterCurve, reg.HoleCurves);
                gen.Cutter_d = base.ToolDiameter.Cached;
                gen.Max_engagement = base.ToolDiameter.Cached * this.StepOver.Cached;
                gen.Min_engagement = base.ToolDiameter.Cached * this.StepOver.Cached * this._min_stepover_percentage.Cached;

                Pocket_generator_emits to_emit = Pocket_generator_emits.LEADIN_SPIRAL | Pocket_generator_emits.BRANCH_ENTRY;
//                    if (get_z_layers().Length > 1)
//                        to_emit |= Pocket_generator_emits.RETURN_TO_BASE;
                gen.Emit_options = to_emit;

                gen.Startpoint = (Point2F)StartPoint.Cached;
                gen.Margin = base.RoughingClearance.Cached;

                // XXX: for now
                gen.Mill_direction = RotationDirection.CCW;
                _path = gen.run();                

//
//                  for (int k = 0; k < path.Count; k++)
//                  {
//                      Polyline p;
//                      Entity e = path[k];
//                      if (e is Arc)
//                          p = ((Arc)e).ToPolyline();
//                      else if (e is Line)
//                          p = ((Line)e).ToPolyline();
//                      else
//                          p = (Polyline)e;
//
//                      ToolpathItem tpi = new ToolpathItem(0, 0, shapeList[0].EntityID, -1, p, RotationDirection.Unknown, Point3F.Undefined, 0.0);
//                      toolpathSequence.Add(tpi);
//                  }

//
//  				regionFiller_OffsetFiller.StepOver = base.ToolDiameter.Cached * this.StepOver.Cached;
//  				regionFiller_OffsetFiller.Margin = base.ToolDiameter.Cached / 2.0 + base.RoughingClearance.Cached;
//  				regionFiller_OffsetFiller.MinimumShapeLength = base.ToolDiameter.Cached * this.StepOver.Cached / 100.0;
//  				regionFiller_OffsetFiller.FinishStepover = this.FinishStepover.Cached;
//  				if (regionFiller_OffsetFiller.GenerateFill() && regionFiller_OffsetFiller.FillShapes != null && regionFiller_OffsetFiller.FillShapes.Count > 0)
//  				{
//  					for (int j = 0; j < regionFiller_OffsetFiller.FillShapes.Count; j++)
//  					{
//  						ShapeListItem shapeListItem2 = regionFiller_OffsetFiller.FillShapes[j];
//  						RegionFill_OffsetItem regionFill_OffsetItem = regionFiller_OffsetFiller.OffsetIndex[j];
//  						if (shapeListItem2.Shape.ID != 0)
//  						{
//  							shapeListItem2.Shape.ID = 0;
//  						}
//  						shapeListItem2.Shape.Update();
//  						if (shapeListItem2.Shape is Polyline)
//  						{
//  							RotationDirection rotationDirection = ((Polyline)shapeListItem2.Shape).Direction;
//  							if (regionFill_OffsetItem.SourceEntity.SubItem2 == -1)
//  							{
//  								rotationDirection = -rotationDirection;
//  							}
//  							ToolpathItem item = new ToolpathItem(0, regionFill_OffsetItem.OffsetIndex, regionFill_OffsetItem.SourceEntity, -1, (Polyline)shapeListItem2.Shape, rotationDirection, Point3F.Undefined, 0.0);
//  							toolpathSequence.Add(item);
//  						}
//  						else if (shapeListItem2.Shape is Region)
//  						{
//  							Polyline outerCurve = ((Region)shapeListItem2.Shape).OuterCurve;
//  							RotationDirection rotationDirection2 = outerCurve.Direction;
//  							if (regionFill_OffsetItem.SourceEntity.SubItem2 == -1)
//  							{
//  								rotationDirection2 = -rotationDirection2;
//  							}
//  							ToolpathItem item2 = new ToolpathItem(0, regionFill_OffsetItem.OffsetIndex, regionFill_OffsetItem.SourceEntity, -1, outerCurve, rotationDirection2, Point3F.Undefined, 0.0);
//  							toolpathSequence.Add(item2);
//  							int num = 0;
//  							Polyline[] holeCurves = ((Region)shapeListItem2.Shape).HoleCurves;
//  							for (int k = 0; k < holeCurves.Length; k++)
//  							{
//  								Polyline toolpath = holeCurves[k];
//  								EntityIdentifier sourceEntity = regionFill_OffsetItem.SourceEntity;
//  								sourceEntity.SubItem2 = num++;
//  								ToolpathItem item3 = new ToolpathItem(0, regionFill_OffsetItem.OffsetIndex, sourceEntity, 0, toolpath, -rotationDirection2, Point3F.Undefined, 0.0);
//  								toolpathSequence.Add(item3);
//  							}
//  						}
//  					}
//  				}
//  				base.Toolpaths2 = toolpathSequence;
//  				base.Toolpaths2.ToolDiameter = base.ToolDiameter.Cached;
//  				this._GenerateToolpathDepthLevels();
//  				base.Toolpaths2.BuildSimpleCutOrder();
//  				//double distanceThreshold = this.GetDistanceThreshold();
//                  double distanceThreshold = this.ToolDiameter.Cached * 0.5;
//  				double minimumSize = distanceThreshold * 0.25;
//  				base.Toolpaths2.InsertLeadIns(this.LeadInMove.Cached, base.PlungeFeedrate.Cached, base.CutFeedrate.Cached, base.StockSurface.Cached, distanceThreshold, minimumSize, this.DepthIncrement.Cached, this.TargetDepth.Cached, this.GetZLayers());
//  				base.Toolpaths2.InsertLeadOuts(this.LeadOutMove.Cached, base.PlungeFeedrate.Cached, base.CutFeedrate.Cached, base.StockSurface.Cached, distanceThreshold, minimumSize, this.DepthIncrement.Cached, this.TargetDepth.Cached, this.GetZLayers());
//  				base.Toolpaths2.DetectRapids(this, this.GetDistanceThreshold());
//  				bool leadin_cutwidths = false;
//  				if ((this.LeadInMove.Cached != null && this.LeadInMove.Cached.LeadInType == LeadInTypeOptions.Tangent) || (this.LeadOutMove.Cached != null && this.LeadOutMove.Cached.LeadInType == LeadInTypeOptions.Tangent))
//  				{
//  					leadin_cutwidths = true;
//  				}
//  				base.Toolpaths2.CalculateCutWidths(base.ToolDiameter.Cached, leadin_cutwidths);
//  				base.ToolpathAnalysis();
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

        public override void PostProcess(MachineOpToGCode gcg)
        {
        	gcg.DefaultStockHeight = base.StockSurface.Cached;

            if (_path == null) return;

            foreach (double depth in get_z_layers())
            {
                for (int eidx = 0; eidx < _path.Count; eidx++)
                {
                    Entity e = _path[eidx];
                    Polyline p;
                    if (e is Arc)
                        p = ((Arc)e).ToPolyline();
                    else if (e is Line)
                        p = ((Line)e).ToPolyline();
                    else
                        p = (Polyline)e;

                    if (eidx == 0)
                    {
                        gcg.AppendPolyLine(p, depth);
                        continue;
                    }

                    if (e is Polyline)
                    {
                        for (int i = 0; i < p.Points.Count; i++)
                        {
                            Point3F pt = new Point3F(p.Points[i].Point.X, p.Points[i].Point.Y, depth);
                            gcg.ApplyGCodeOrigin(ref pt);
                            gcg.AppendMove("g1", pt.X, pt.Y, pt.Z, _chord_feedrate.Cached);
                        }
                    }
                    else
                    {
                        Point3F pt = new Point3F(p.Points[0].Point.X, p.Points[0].Point.Y, depth);
                        gcg.ApplyGCodeOrigin(ref pt);
                        gcg.AppendMove("g1", pt.X, pt.Y, pt.Z, _chord_feedrate.Cached);
                        gcg.AppendPolyLine(p, depth);
                    }
                }
            }
        }

        public override void Paint(ICADView iv, Display3D d3d, Color arccolor, Color linecolor, bool selected)
        {
        	this._CADFile = iv.CADFile;

            foreach (double depth in get_z_layers())
            {
                foreach (Entity e in _path)
                {
                    Color arcColor = arccolor;
                    Color lineColor = linecolor;
                    d3d.LineWidth = (float)1;

                    Polyline p;

                    if (e is Arc)
                        p = ((Arc)e).ToPolyline();
                    else if (e is Polyline)
                        p = ((Polyline)e);
                    else if (e is Line)
                        p = ((Line)e).ToPolyline();
                    else
                        continue;


                    Matrix4x4F matrix4x4F = new Matrix4x4F();
                    matrix4x4F.Translate(0.0, 0.0, depth);

                    if (base.Transform.Cached != null)
                        matrix4x4F *= base.Transform.Cached;

                    d3d.ModelTransform = matrix4x4F;
                    p.Paint(d3d, arcColor, lineColor);
                    base.PaintDirectionVector(iv, p, d3d, matrix4x4F);
                    //base.PaintToolpathNormal(iv, toolpathItem, toolpath, d3d, matrix4x4F);
                }
            }

//            if (_path)
//            {
//                Color color = Color.Empty;
//                Color color2 = Color.Empty;
//                int num = -1;
//                int num2 = -1;
//                int num3 = -1;
//                int num4 = -1;
////      		if (CamBamUI.MainUI.ToolpathFilterVisible)
////      		{
////      			ToolpathViewFilter toolpathViewFilter = CamBamConfig.Defaults.ToolpathViewFilter;
////      			if (toolpathViewFilter.FilterZDepth)
////      			{
////      				num3 = toolpathViewFilter.ZDepth;
////      			}
////      			if (toolpathViewFilter.FilterToolpathIndex)
////      			{
////      				num4 = toolpathViewFilter.ToolpathIndex;
////      			}
////      			if (toolpathViewFilter.UseCutToolpathColor && (toolpathViewFilter.FilterZDepth || toolpathViewFilter.FilterToolpathIndex))
////      			{
////      				color = toolpathViewFilter.CutToolpathColor;
////      				num = toolpathViewFilter.CutToolpathLineWidth;
////      			}
////      			if (toolpathViewFilter.UseToolpathColor && (toolpathViewFilter.FilterZDepth || toolpathViewFilter.FilterToolpathIndex))
////      			{
////      				color2 = toolpathViewFilter.ToolpathColor;
////      				num2 = toolpathViewFilter.ToolpathLineWidth;
////      			}
////      		}
//                int num5 = 0;
//                if (base.Toolpaths2.CutOrder != null)
//                {
//                    using (List<int>.Enumerator enumerator = base.Toolpaths2.CutOrder.GetEnumerator())
//                    {
//                        while (enumerator.MoveNext())
//                        {
//                            int current = enumerator.Current;
//                            ToolpathItem toolpathItem = base.Toolpaths2.Toolpaths[current];
//                            Color arcColor = arccolor;
//                            Color lineColor = linecolor;
//                            d3d.LineWidth = 1f;
//                            if (num3 > -1)
//                            {
//                                if (toolpathItem.DepthIndex < num3)
//                                {
//                                    if (color.IsEmpty)
//                                    {
//                                        continue;
//                                    }
//                                    arcColor = color;
//                                    lineColor = color;
//                                    d3d.LineWidth = (float)num;
//                                }
//                                else
//                                {
//                                    if (toolpathItem.DepthIndex > num3)
//                                    {
//                                        continue;
//                                    }
//                                    if (!color2.IsEmpty)
//                                    {
//                                        arcColor = color2;
//                                        lineColor = color2;
//                                        d3d.LineWidth = (float)num2;
//                                    }
//                                }
//                            }
//                            if (num4 > -1)
//                            {
//                                if (num5 < num4)
//                                {
//                                    if (color.IsEmpty)
//                                    {
//                                        num5++;
//                                        continue;
//                                    }
//                                    arcColor = color;
//                                    lineColor = color;
//                                    d3d.LineWidth = (float)num;
//                                }
//                                else
//                                {
//                                    if (num5 > num4)
//                                    {
//                                        num5++;
//                                        continue;
//                                    }
//                                    if (!color2.IsEmpty)
//                                    {
//                                        arcColor = color2;
//                                        lineColor = color2;
//                                        d3d.LineWidth = (float)num2;
//                                    }
//                                }
//                                num5++;
//                            }
//                            Polyline toolpath = toolpathItem.Toolpath;
//                            Matrix4x4F matrix4x4F = new Matrix4x4F();
//                            matrix4x4F.Translate(0.0, 0.0, toolpathItem.ZOffset);
//                            if (base.Transform.Cached != null)
//                            {
//                                matrix4x4F *= base.Transform.Cached;
//                            }
//                            d3d.ModelTransform = matrix4x4F;
//                            toolpath.Paint(d3d, arcColor, lineColor);
//                            d3d.LineWidth = 1f;
//                            base.PaintDirectionVector(iv, toolpath, d3d, matrix4x4F);
//                            base.PaintToolpathNormal(iv, toolpathItem, toolpath, d3d, matrix4x4F);
//                        }
//                        goto IL_39A;
//                    }
//                }
//                for (int i = 0; i < base.Toolpaths2.Toolpaths.Count; i++)
//                {
//                    ToolpathItem toolpathItem2 = base.Toolpaths2.Toolpaths[i];
//                    if ((num3 <= -1 || toolpathItem2.DepthIndex == num3) && (num4 <= -1 || num5 == num4))
//                    {
//                        num5++;
//                        Polyline toolpath2 = toolpathItem2.Toolpath;
//                        Matrix4x4F matrix4x4F2 = new Matrix4x4F();
//                        matrix4x4F2.Translate(0.0, 0.0, toolpathItem2.ZOffset);
//                        if (base.Transform.Cached != null)
//                        {
//                            matrix4x4F2 *= base.Transform.Cached;
//                        }
//                        d3d.ModelTransform = matrix4x4F2;
//                        toolpath2.Paint(d3d, arccolor, linecolor);
//                        base.PaintDirectionVector(iv, toolpath2, d3d, matrix4x4F2);
//                        base.PaintToolpathNormal(iv, toolpathItem2, toolpath2, d3d, matrix4x4F2);
//                    }
//                }
//                IL_39A:
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
//            }
        	if (selected)
        	{
        		base.PaintStartPoint(iv, d3d);
        	}
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