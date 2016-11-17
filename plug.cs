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

            Pocket_generator gen = new Pocket_generator(reg.OuterCurve, reg.HoleCurves);
            gen.Cutter_d = cutter_d;
            gen.Max_engagement = max_engagement;
            gen.Min_engagement = max_engagement / 2.0;
            gen.Mill_direction = RotationDirection.CCW;
            gen.Emit_options = Pocket_path_item_type.BRANCH_ENTRY | Pocket_path_item_type.SEGMENT | Pocket_path_item_type.LEADIN_SPIRAL | Pocket_path_item_type.DEBUG_MAT;
            List<Pocket_path_item> path = gen.run();

            Host.log("path generated");

            foreach (Pocket_path_item item in path)
            {
                CamBamUI.MainUI.ActiveView.CADFile.ActiveLayer.Entities.Add(item);
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

        static private void insert_in_top_menu(CamBamUI ui, ToolStripMenuItem entry)
        {
            for (int i = 0; i < ui.Menus.mnuMachining.DropDownItems.Count; ++i)
            {
                ToolStripItem tsi = ui.Menus.mnuMachining.DropDownItems[i];
                if (tsi is ToolStripSeparator || i == ui.Menus.mnuMachining.DropDownItems.Count - 1)
                {
                    ui.Menus.mnuMachining.DropDownItems.Insert(i, entry);
                    return;
                }
            }
        }

        static private void insert_in_context_menu(CamBamUI ui, ToolStripMenuItem entry)
        {
            foreach (ToolStripItem tsi in ui.ViewContextMenus.ViewContextMenu.Items)
            {
                if (tsi is ToolStripMenuItem && tsi.Name == "machineToolStripMenuItem")
                {
                    ToolStripMenuItem tsmi = (ToolStripMenuItem)tsi;
                    for (int i = 0; i < tsmi.DropDownItems.Count; ++i)
                    {
                        if (tsmi.DropDownItems[i] is ToolStripSeparator || i == tsmi.DropDownItems.Count - 1)
                        {
                            tsmi.DropDownItems.Insert(i, entry);
                            return;
                        }
                    }
                }
            }
        }

        static private void insert_in_toolbar(ToolStripButton button)
        {
            foreach (Control c in ThisApplication.TopWindow.Controls)
            {
                if (c is ToolStripContainer)
                {
                    foreach (Control cc in ((ToolStripContainer)c).TopToolStripPanel.Controls)
                    {
                        if (cc is CAMToolStrip)
                        {
                            CAMToolStrip strip = (CAMToolStrip)cc;
                            strip.Items.Add(button);
                            return;
                        }
                    }
                }
            }
        }

        static void on_load(object sender, EventArgs e)
        {
            ToolStripButton button = new ToolStripButton();
            button.ToolTipText = "Trochoidal Pocket";
            button.Click += mop_onclick;
            button.Image = resources.cam_trochopock;

            insert_in_toolbar(button);
        }

        public static void InitPlugin(CamBamUI ui)
        {
            ToolStripMenuItem popup = new ToolStripMenuItem("MAT DEBUG");
            popup.Click += popup_handler;
            ui.Menus.mnuPlugins.DropDownItems.Add(popup);

            ToolStripMenuItem menu_entry;

            menu_entry = new ToolStripMenuItem();
            menu_entry.Text = "Trochoidal Pocket";
            menu_entry.Click += mop_onclick;
            menu_entry.Image = resources.cam_trochopock;

            insert_in_top_menu(ui, menu_entry);

            menu_entry = new ToolStripMenuItem();
            menu_entry.Text = "Trochoidal Pocket";
            menu_entry.Click += mop_onclick;
            menu_entry.Image = resources.cam_trochopock;

            insert_in_context_menu(ui, menu_entry);

            // defer attachment to toolbar until the full load
            ThisApplication.TopWindow.Load += on_load;

            if (CADFile.ExtraTypes == null)
				CADFile.ExtraTypes = new List<Type>();
			CADFile.ExtraTypes.Add(typeof(Mop_matmill));

			Mop_matmill o = new Mop_matmill();
			XmlSerializer xmlSerializer = new XmlSerializer(typeof(Mop_matmill));
			MemoryStream stream = new MemoryStream();
			xmlSerializer.Serialize(stream, o);
        }
    }
}