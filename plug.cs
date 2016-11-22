using System;
using System.Windows.Forms;
using System.Collections.Generic;
using System.Xml.Serialization;
using System.IO;

using CamBam;
using CamBam.UI;
using CamBam.CAD;
using CamBam.Geom;

namespace Medial_demo
{
    class Host
    {
        static public void log(string s, params object[] args)
        {
            ThisApplication.AddLogMessage(4, s, args);
        }
        static public void warn(string s, params object[] args)
        {
            ThisApplication.AddLogMessage("warning: " + s, args);
        }
        static public void err(string s, params object[] args)
        {
            ThisApplication.AddLogMessage("error: " + s, args);
        }
    }

    public class Matmill_plugin
    {

        static private void gen_pocket(ShapeListItem shape)
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
                return;
            }

            Pocket_generator gen = new Pocket_generator(outline, islands);
            // NOTE: metric
            gen.Sample_step = 0.1;          // 0.1 mm 
            gen.General_tolerance = 0.001;  // 0.001 mm
            List<Polyline> medial_axis = gen.run();

            if (medial_axis == null)
                return;

            foreach (Polyline p in medial_axis)
                CamBamUI.MainUI.ActiveView.CADFile.ActiveLayer.Entities.Add(p);
        }


        private static void onclick(object sender, EventArgs ars)
        {
			if (!PolylineUtils.ConfirmSelected(CamBamUI.MainUI.ActiveView))
				return;

            ShapeList shapes = new ShapeList();
            shapes.ApplyTransformations = true;
            shapes.AddEntities(new List<Entity>((Entity[]) CamBamUI.MainUI.ActiveView.SelectedEntities));
            shapes = shapes.DetectRegions();

            // remove open polylines
            for (int i = shapes.Count - 1; i >= 0; i--)
            {
                if (shapes[i].Shape is Polyline && !((Polyline)shapes[i].Shape).Closed)
                    shapes.RemoveAt(i);
            }

            if (shapes.Count == 0)
                return;

            // add undo point
            CamBamUI.MainUI.ActiveView.SuspendRefresh();
            CamBamUI.MainUI.ActiveView.CADFile.Modified = true;
            CamBamUI.MainUI.UndoBuffer.AddUndoPoint("Medial Demo");
            CamBamUI.MainUI.ActiveView.CADFile.EnsureActiveLayer(true);
            CamBamUI.MainUI.UndoBuffer.Add(CamBamUI.MainUI.ActiveView.CADFile.ActiveLayer.Entities);

            // generate
            foreach (ShapeListItem shape in shapes)
                gen_pocket(shape);

            CamBamUI.MainUI.ActiveView.ResumeRefresh();
            CamBamUI.MainUI.ActiveView.UpdateViewport();
        }

        public static void InitPlugin(CamBamUI ui)
        {
            ToolStripMenuItem handler = new ToolStripMenuItem("Medial Demo");
            handler.Click += onclick;
            ui.Menus.mnuPlugins.DropDownItems.Add(handler);
        }
    }
}