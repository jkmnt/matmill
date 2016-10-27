using System.Windows.Forms;

using CamBam.UI;
using CamBam.CAD;
using CamBam.Geom;

namespace Tree4
{
    public class T4_nearest_debugger: EditMode
    {
        System.Drawing.Point cursor = new System.Drawing.Point(0, 0);
        T4 _t4;

        Point3F to_drawing_point(System.Drawing.Point pt)
        {
            return this._ActiveView.ScreenToDrawing(new System.Drawing.PointF((float)pt.X, (float)pt.Y)); ;
        }

        public T4_nearest_debugger(ICADView iv, T4 t4) : base(iv)
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

        double get_mic_radius(Point2F pt)
        {
            double radius = double.MaxValue;
            foreach (object item in _t4.Get_nearest_objects(pt.X, pt.Y))
            {
                double dist = 0;
                if (item is Line2F)
                    ((Line2F)item).NearestPoint(pt, ref dist);
                else
                    ((Arc2F)item).NearestPoint(pt, ref dist);
                if (dist < radius)
                    radius = dist;
            }

            return radius;
        }

        public override void OnPaint(ICADView iv, Display3D d3d)
        {
            if (base.ReturnStatus != EditMode.ReturnStatusCode.Running) return;

            d3d.ModelTransform = Matrix4x4F.Identity;
            d3d.LineColor = System.Drawing.Color.Cyan;

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

            double radius = get_mic_radius((Point2F)point);
            Circle mic = new Circle(point, radius);

            d3d.LineColor = System.Drawing.Color.Magenta;
            mic.Paint(d3d);

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

    public class T4_collider_debugger : EditMode
    {
        System.Drawing.Point cursor = new System.Drawing.Point(0, 0);
        T4 _t4;
        double _width = 10;
        double _height = 10;

        Point3F to_drawing_point(System.Drawing.Point pt)
        {
            return this._ActiveView.ScreenToDrawing(new System.Drawing.PointF((float)pt.X, (float)pt.Y));
            ;
        }

        public T4_collider_debugger(ICADView iv, T4 t4) : base(iv)
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
            if (base.ReturnStatus != EditMode.ReturnStatusCode.Running)
                return;

            d3d.ModelTransform = Matrix4x4F.Identity;
            d3d.LineColor = System.Drawing.Color.Cyan;

            Point3F point = to_drawing_point(this.cursor);

            T4_rect checkbox = new T4_rect(point.X, point.Y, point.X + _width, point.Y + _height);

            foreach (T4_rect rect in _t4.Get_colliding_obj_rects(checkbox))
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

            d3d.LineColor = System.Drawing.Color.Magenta;

            Point3F pr0 = new Point3F(checkbox.Xmin, checkbox.Ymin, 0);
            Point3F pr1 = new Point3F(checkbox.Xmin, checkbox.Ymax, 0);
            Point3F pr2 = new Point3F(checkbox.Xmax, checkbox.Ymax, 0);
            Point3F pr3 = new Point3F(checkbox.Xmax, checkbox.Ymin, 0);
            d3d.DrawLine(pr0, pr1);
            d3d.DrawLine(pr1, pr2);
            d3d.DrawLine(pr2, pr3);
            d3d.DrawLine(pr3, pr0);

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