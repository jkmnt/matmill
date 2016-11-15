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
//using System.Drawing;
using System.Xml.Serialization;

namespace Matmill
{
    // XXX: inherit from MOPPocket to give toolpath generator access to the hardcoded MOPPocket fields
    [Serializable]
    public class Mop_matmill : MOPPocket
    {
        protected CBValue<double> _min_stepover_percentage;
        protected CBValue<double> _chord_feedrate;

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
        public new CBValue<bool> CollisionDetection
        {
            get { return base.CollisionDetection; }
            set { }
        }

        [Browsable(false), XmlIgnore]
        public new CBValue<OptimisationModes> OptimisationMode
        {
            get { return base.OptimisationMode; }
            set { }
        }

        [Browsable(false), XmlIgnore]
        public new CBValue<RegionFillStyles> RegionFillStyle
        {
            get { return base.RegionFillStyle; }
            set { }
        }

        // this puppy is hidden now
        /*        [Browsable(false), XmlIgnore]
                public new CBValue<double> MaxCrossoverDistance
                {
                    get
                    {
                        return base.MaxCrossoverDistance;
                    }
                    set
                    {
                        base.MaxCrossoverDistance = value;
                    }
                }
        */

        [CBKeyValue, Category("Feedrates"),
                     DefaultValue(typeof(CBValue<double>), "0.1"),
                     Description("Feedrate for the slice chords and traverse inside the milled pocket"),
                     DisplayName("Chord Feedrate")]
		public CBValue<double> Chord_feedrate
		{
			get
			{
                return _chord_feedrate;
			}
			set
			{
                _chord_feedrate = value;
                this.StepoverFeedrate.SetValue(new StepoverFeedrateInfo(_chord_feedrate.Cached));
            }
		}

        [CBKeyValue, Category("Step Over"),
                     //DefaultValue(typeof(CBValue<double>), "Default"),
                     DefaultValue(typeof(CBValue<double>), "0.1"),
                     Description("Minimum allowed stepover as a percentage of the nominal stepover (0.1 - 0.9)"),
                     DisplayName("Minimum Stepover")]
		public CBValue<double> Min_stepover
		{
			get
			{
                return this._min_stepover_percentage;
            }
			set
			{
                this._min_stepover_percentage = value;
			}
		}

		protected override void _GenerateToolpathsWorker()
		{
			try
			{
				base.Toolpaths2 = null;
				ToolpathSequence toolpathSequence = new ToolpathSequence(this);
				GC.Collect();
				base.UpdateGeometryExtrema(this._CADFile);
				this._CADFile.MachiningOptions.UpdateGeometryExtrema(this._CADFile);

				ShapeList shapeList = new ShapeList();
				shapeList.ApplyTransformations = true;
				shapeList.AddEntities(this._CADFile, base.PrimitiveIds);
				shapeList = shapeList.DetectRegions();

				bool has_opened_polylines = false;
				for (int i = shapeList.Count - 1; i >= 0; i--)
				{
					ShapeListItem shapeListItem = shapeList[i];
					if (shapeListItem.Shape is Polyline)
					{
						Polyline p = (Polyline)shapeListItem.Shape;
						if (! p.Closed)
						{
							has_opened_polylines = true;
							shapeList.RemoveAt(i);
						}
					}
				}
				if (has_opened_polylines)
				{
                    Host.warn("Found open polylines: ignoring");
					this.MachineOpStatus = MachineOpStatus.Warnings;
				}
				if (shapeList.Count != 0)
				{
					CamBam.CAD.Region[] regs = CamBam.CAD.Region.CreateFromPolylines(shapeList.ToPolylines().ToArray());

					//if (regs.Length != 1) return;

					CamBam.CAD.Region reg = regs[0];

                    Pocket_generator gen = new Pocket_generator(reg);
                    gen.Cutter_d = base.ToolDiameter.Cached;
                    gen.Max_engagement = base.ToolDiameter.Cached * this.StepOver.Cached;
                    gen.Min_engagement = base.ToolDiameter.Cached * this.StepOver.Cached * this._min_stepover_percentage.Cached;
                    gen.Return_to_base = this.GetZLayers().Length > 1;
                    gen.Startpoint = (Point2F)StartPoint.Cached;
                    gen.Margin = base.RoughingClearance.Cached;

                    // XXX: for now
                    gen.Mill_direction = RotationDirection.CCW;

                    List<Entity> path = gen.run();

                    for (int k = 0; k < path.Count; k++)
                    {
                        Polyline p;
                        Entity e = path[k];
                        if (e is Arc)
                            p = ((Arc)e).ToPolyline();
                        else if (e is Line)
                            p = ((Line)e).ToPolyline();
                        else
                            p = (Polyline)e;

                        ToolpathItem tpi = new ToolpathItem(0, 0, shapeList[0].EntityID, -1, p, RotationDirection.Unknown, Point3F.Undefined, 0.0);
                        toolpathSequence.Add(tpi);
                    }

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
					base.Toolpaths2 = toolpathSequence;
					base.Toolpaths2.ToolDiameter = base.ToolDiameter.Cached;
					this._GenerateToolpathDepthLevels();
					base.Toolpaths2.BuildSimpleCutOrder();
					//double distanceThreshold = this.GetDistanceThreshold();
                    double distanceThreshold = this.ToolDiameter.Cached * 0.5;
					double minimumSize = distanceThreshold * 0.25;
					base.Toolpaths2.InsertLeadIns(this.LeadInMove.Cached, base.PlungeFeedrate.Cached, base.CutFeedrate.Cached, base.StockSurface.Cached, distanceThreshold, minimumSize, this.DepthIncrement.Cached, this.TargetDepth.Cached, this.GetZLayers());
					base.Toolpaths2.InsertLeadOuts(this.LeadOutMove.Cached, base.PlungeFeedrate.Cached, base.CutFeedrate.Cached, base.StockSurface.Cached, distanceThreshold, minimumSize, this.DepthIncrement.Cached, this.TargetDepth.Cached, this.GetZLayers());
					base.Toolpaths2.DetectRapids(this, this.GetDistanceThreshold());
					bool leadin_cutwidths = false;
					if ((this.LeadInMove.Cached != null && this.LeadInMove.Cached.LeadInType == LeadInTypeOptions.Tangent) || (this.LeadOutMove.Cached != null && this.LeadOutMove.Cached.LeadInType == LeadInTypeOptions.Tangent))
					{
						leadin_cutwidths = true;
					}
					base.Toolpaths2.CalculateCutWidths(base.ToolDiameter.Cached, leadin_cutwidths);
					base.ToolpathAnalysis();
					if (this.MachineOpStatus == MachineOpStatus.Unknown)
					{
						this.MachineOpStatus = MachineOpStatus.OK;
					}
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

        public Mop_matmill()
        {
            //MaxCrossoverDistance.SetValue(1000.0);
        }

        public Mop_matmill(CADFile CADFile, ICollection<Entity> plist) : base(CADFile, plist)
        {
            //MaxCrossoverDistance.SetValue(1000.0);
        }

    }

}