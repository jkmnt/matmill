using System.Windows.Forms;

using CamBam.UI;
using CamBam.CAD;
using CamBam.Geom;

namespace Tree4
{
    public class T4_debugger: EditMode
    {
        System.Drawing.Point cursor = new System.Drawing.Point(0, 0);
        T4 _t4;

        Point3F to_drawing_point(System.Drawing.Point pt)
        {
            return this._ActiveView.ScreenToDrawing(new System.Drawing.PointF((float)pt.X, (float)pt.Y)); ;
        }

        public T4_debugger(ICADView iv, T4 t4) : base(iv)
        {                  
            this._t4 = t4;
            this.MarkFileModified = false;

            foreach (T4_rect rect in _t4.Traverse_rects())
            {
                PolyRectangle pr = new PolyRectangle(new Point3F(rect.Xmin, rect.Ymin, 0), rect.W, rect.H);
                CamBamUI.MainUI.ActiveView.CADFile.ActiveLayer.Entities.Add(pr);
            }

            CamBamUI.MainUI.ActiveView.SetEditMode(this, true);
            CamBamUI.MainUI.ActiveView.RepaintEditMode();
        }

        public override void OnPaint(ICADView iv, Display3D d3d)
        {
            if (base.ReturnStatus != EditMode.ReturnStatusCode.Running) return;

            //Layer layer = this._ActiveView.CADFile.EnsureActiveLayer(true);
            d3d.ModelTransform = Matrix4x4F.Identity;
            d3d.LineColor = System.Drawing.Color.Cyan;
            //d3d.DrawPoint(to_drawing_point(this.cursor), (float)this.pixel_tolerance);

            Point3F point = to_drawing_point(this.cursor);
            foreach (T4_rect rect in _t4.Get_nearest_obj_rects(point.X, point.Y))
            {
                Point3F p0 = new Point3F(rect.Xmin, rect.Ymin, 0);
                Point3F p1 = new Point3F(rect.Xmin, rect.Ymax, 0);
                Point3F p2 = new Point3F(rect.Xmax, rect.Ymax, 0);
                Point3F p3 = new Point3F(rect.Xmax, rect.Ymin, 0);
                d3d.DrawLine(p0, p1);
                d3d.DrawLine(p1, p2);
                d3d.DrawLine(p2, p3);
                d3d.DrawLine(p3, p0);

            }

            base.OnPaint(iv, d3d);
        }

        public override bool OnKeyDown(object sender, KeyEventArgs e)
        {
            if (e.KeyCode == Keys.Return || e.KeyCode == Keys.Escape)
            {
                this.ReturnCancel();
                return true;
            }
            return false;
        }

        public override bool OnMouseMove(object sender, MouseEventArgs e)
        {
            this.cursor = e.Location;
            this._ActiveView.RepaintEditMode();
            return base.OnMouseMove(sender, e);
        }
    }
}