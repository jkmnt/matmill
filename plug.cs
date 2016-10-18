using System;
using System.Windows.Forms;
using System.Collections.Generic;

using CamBam;
using CamBam.UI;
using CamBam.CAD;

namespace Matmill
{
    public class Host
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

            /*          ret = ThisApplication.PromptForValue("Enter sample distance", "s", typeof(double), "0.5");
                        if (ret == null) return;
                        double sample_distance = Math.Abs((double)ret);
            */
            double sample_distance = max_engagement / 10.0;
            //double sample_distance = cutter_d / 10.0;

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
            gen.cutter_d = cutter_d;
            gen.max_engagement = max_engagement;
            gen.sample_distance = sample_distance;
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

        static void debug_handler(object sender, EventArgs ars)
        {            
            Host.log("hello");

            if (CamBamUI.MainUI.ActiveView.CADFile.ActiveLayer == null)
                return;

            object[] selection = CamBamUI.MainUI.ActiveView.SelectedEntities;
            if (selection.Length == 0)
                return;

            List<Polyline> polys = new List<Polyline>();

            foreach (object obj in selection)
            {
                if (obj is Polyline)
                    polys.Add((Polyline)obj);
            }

            if (polys.Count == 0)
                return;

            CamBam.CAD.Region[] regs = CamBam.CAD.Region.CreateFromPolylines(polys.ToArray());

            if (regs.Length != 1)
                return;

            CamBam.CAD.Region reg = regs[0];

            CamBamUI.MainUI.ActiveView.SuspendRefresh();
            CamBamUI.MainUI.ActiveView.CADFile.Modified = true;
            CamBamUI.MainUI.UndoBuffer.AddUndoPoint("matmill");
            CamBamUI.MainUI.ActiveView.CADFile.EnsureActiveLayer(true);
            CamBamUI.MainUI.UndoBuffer.Add(CamBamUI.MainUI.ActiveView.CADFile.ActiveLayer.Entities);

            Pocket_generator gen = new Pocket_generator(reg);            
            gen.Debug_t4();

            CamBamUI.MainUI.ActiveView.ResumeRefresh();
            CamBamUI.MainUI.ActiveView.UpdateViewport();

            Host.log("done");
        }

        public static void InitPlugin(CamBamUI ui)
        {
            ToolStripMenuItem popup = new ToolStripMenuItem("MAT");
            popup.Click += popup_handler;
            ui.Menus.mnuPlugins.DropDownItems.Add(popup);

            ToolStripMenuItem popup2 = new ToolStripMenuItem("MAT debug");
            popup2.Click += debug_handler;
            ui.Menus.mnuPlugins.DropDownItems.Add(popup2);
        }
    }
}