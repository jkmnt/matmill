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

using Matmill;

namespace Trochopock
{
    [Serializable]
    public class MOPTrochoprof : Sliced_mop, IIcon
    {
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

        private Sliced_path gen_profile(Polyline poly)
        {
            Engrave_generator gen = new Engrave_generator(poly);

            gen.General_tolerance = is_inch_units() ? 0.001 / 25.4 : 0.001;
            gen.Tool_d = base.ToolDiameter.Cached;
            gen.Max_ted = base.ToolDiameter.Cached * _stepover.Cached;
            //gen.Min_ted = base.ToolDiameter.Cached * _stepover.Cached * _min_stepover_percentage;

            //gen.Startpoint = (Point2F)base.StartPoint.Cached;
            //gen.Margin = base.RoughingClearance.Cached;

            // XXX: for now
            gen.Slice_radius = base.ToolDiameter.Cached / 2;

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

                List<Polyline> polys = new List<Polyline>();

                foreach (ShapeListItem shape in shapes)
                {
                    if (shape.Shape is Polyline)
                    {
                        polys.Add((Polyline)shape.Shape);
                    }
                }

//              bool found_opened_polylines = false;
//              for (int i = shapes.Count - 1; i >= 0; i--)
//              {
//                  if (shapes[i].Shape is Polyline && ! ((Polyline)shapes[i].Shape).Closed)
//                  {
//                      found_opened_polylines = true;
//                      shapes.RemoveAt(i);
//                  }
//              }
//              if (found_opened_polylines)
//              {
//                  Host.warn("ignoring open polylines");
//                  base.MachineOpStatus = MachineOpStatus.Warnings;
//              }

                List<Sliced_path> trajectories = new List<Sliced_path>();

                foreach (Polyline p in polys)
                {
                    Sliced_path traj = gen_profile(p);
                    if (traj != null)
                        trajectories.Add(traj);
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
        }

        public MOPTrochoprof()
        {
        }

        public MOPTrochoprof(CADFile CADFile, ICollection<Entity> plist) : base(CADFile, plist)
        {
        }
    }

}
