using System;
using System.Windows.Forms;
using System.Collections.Generic;
using System.Xml.Serialization;
using System.IO;

using CamBam;
using CamBam.UI;
using CamBam.CAD;
using CamBam.Geom;

namespace Matmill
{
    class Host
    {
        static public void log(string s, params object[] args)
        {
            ThisApplication.AddLogMessage(s, args);
        }
        static public void warn(string s, params object[] args)
        {
            ThisApplication.AddLogMessage("Warning: " + s, args);
        }
        static public void err(string s, params object[] args)
        {
            ThisApplication.AddLogMessage("Error: " + s, args);
        }
        static public void msg(string s, params object[] args)
        {
            ThisApplication.MsgBox(String.Format(s, args));
        }
        static public void sleep(int ms)
        {
            System.Threading.Thread.Sleep(ms);
            System.Windows.Forms.Application.DoEvents();
        }
    }

    public class Matmill_plugin
    {
        static void popup_handler(object sender, EventArgs ars)
        {
            object ret;
            ret = ThisApplication.PromptForValue("Enter cutter diameter", "d", typeof(double), "3");
            if (ret == null) return;
            double cutter_d = Math.Abs((double)ret);

            ret = ThisApplication.PromptForValue("Enter stepover", "s", typeof(double), "0.4");
            if (ret == null) return;
            double max_engagement = Math.Abs((double)ret) * cutter_d;

            Host.log("hello");

            if (CamBamUI.MainUI.ActiveView.CADFile.ActiveLayer == null)
                return;

            object[] selection = CamBamUI.MainUI.ActiveView.SelectedEntities;
            if (selection.Length == 0) return;

            List<Polyline> polys = new List<Polyline>();

            foreach (object obj in selection)
            {
                if (obj is Polyline)
                    polys.Add((Polyline)obj);
            }

            if (polys.Count == 0) return;

            CamBam.CAD.Region[] regs = CamBam.CAD.Region.CreateFromPolylines(polys.ToArray());

            if (regs.Length != 1) return;

            CamBam.CAD.Region reg = regs[0];

            CamBamUI.MainUI.ActiveView.SuspendRefresh();
            CamBamUI.MainUI.ActiveView.CADFile.Modified = true;
            CamBamUI.MainUI.UndoBuffer.AddUndoPoint("matmill");
            CamBamUI.MainUI.ActiveView.CADFile.EnsureActiveLayer(true);
            CamBamUI.MainUI.UndoBuffer.Add(CamBamUI.MainUI.ActiveView.CADFile.ActiveLayer.Entities);

            Pocket_generator gen = new Pocket_generator(reg);
            gen.Cutter_d = cutter_d;
            gen.Max_engagement = max_engagement;
            gen.Min_engagement = max_engagement / 2.0;
            gen.Mill_direction = RotationDirection.CCW;
            //gen.Startpoint = new Point2F(-116, -3);
            List<Entity> path = gen.run();

            Host.log("path generated");


            foreach (Entity e in path)
            {
                CamBamUI.MainUI.ActiveView.CADFile.ActiveLayer.Entities.Add(e);
            }

            CamBamUI.MainUI.ActiveView.ResumeRefresh();
            CamBamUI.MainUI.ActiveView.UpdateViewport();

            Host.log("done");
        }

        static void mop_onclick(object sender, EventArgs ars)
        {
			if (!PolylineUtils.ConfirmSelected(CamBamUI.MainUI.ActiveView))
			{
				return;
			}

			Mop_matmill mop = new Mop_matmill(CamBamUI.MainUI.ActiveView.CADFile, CamBamUI.MainUI.ActiveView.Selection);
            CamBamUI.MainUI.InsertMOP(mop);
        }

        public static void InitPlugin(CamBamUI ui)
        {
            ToolStripMenuItem popup = new ToolStripMenuItem("MAT");
            popup.Click += popup_handler;
            ui.Menus.mnuPlugins.DropDownItems.Add(popup);

            ToolStripMenuItem mop_popup = new ToolStripMenuItem("MAT mill");
            mop_popup.Click += mop_onclick;
            ui.Menus.mnuPlugins.DropDownItems.Add(mop_popup);

            if (CADFile.ExtraTypes == null)
			{
				CADFile.ExtraTypes = new List<Type>();
			}
			CADFile.ExtraTypes.Add(typeof(Mop_matmill));
			Mop_matmill o = new Mop_matmill();
			XmlSerializer xmlSerializer = new XmlSerializer(typeof(Mop_matmill));
			MemoryStream stream = new MemoryStream();
			xmlSerializer.Serialize(stream, o);
        }
    }
}