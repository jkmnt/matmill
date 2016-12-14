using System;
using System.Collections.Generic;

using CamBam.Geom;
using CamBam.CAD;

using Geom;

namespace Matmill
{
    enum Sliced_path_item_type
    {
        SLICE,
        SPIRAL,
        CHORD,
        SMOOTH_CHORD,
        GUIDE,
        SLICE_SHORTCUT,
        RETURN_TO_BASE,
        DEBUG_MEDIAL_AXIS,
    }

    class Sliced_path : List<Sliced_path_item> { }

    class Sliced_path_item: Polyline
    {
        public readonly Sliced_path_item_type Item_type;

        public Sliced_path_item(Sliced_path_item_type type) : base()
        {
            Item_type = type;
        }

        public Sliced_path_item(Sliced_path_item_type type, int i) : base(i)
        {
            Item_type = type;
        }

        public Sliced_path_item(Sliced_path_item_type type, Polyline p) : base(p)
        {
            Item_type = type;
        }

        public void Add(Point2F pt)
        {
            base.Add(new Point3F(pt.X, pt.Y, 0));
        }

        public void Add(Curve curve)
        {
            foreach (Point2F pt in curve.Points)
                this.Add(pt);
        }

        public void Add(Biarc2d biarc, double tolerance)
        {
            if (biarc.Seg1 is Arc2F)
                this.Add((Arc2F)biarc.Seg1, tolerance);
            else
                this.Add((Line2F)biarc.Seg1, tolerance);

            if (biarc.Seg2 is Arc2F)
                this.Add((Arc2F)biarc.Seg2, tolerance);
            else
                this.Add((Line2F)biarc.Seg2, tolerance);
        }
    }

    class Sliced_path_generator
    {
        protected double _general_tolerance;

        public Sliced_path Path = new Sliced_path();
        protected Slice _last_slice = null;

        protected void append_slice(Slice slice)
        {
            // emit segments
            for (int segidx = 0; segidx < slice.Segments.Count; segidx++)
            {
                // connect segments
                if (segidx > 0)
                {
                    Sliced_path_item shortcut = new Sliced_path_item(Sliced_path_item_type.SLICE_SHORTCUT);
                    shortcut.Add(slice.Segments[segidx - 1].P2);
                    shortcut.Add(slice.Segments[segidx].P1);
                    Path.Add(shortcut);
                }

                Sliced_path_item arc = new Sliced_path_item(Sliced_path_item_type.SLICE);
                arc.Add(slice.Segments[segidx], _general_tolerance);
                Path.Add(arc);
            }

            _last_slice = slice;
        }

        protected Sliced_path_item connect_slices(Slice dst, Slice src)
        {
            Sliced_path_item path = new Sliced_path_item(Sliced_path_item_type.CHORD);
            path.Add(src.End);
            path.Add(dst.Start);
            return path;
        }

        public void Append_spiral(Point2F start, Point2F end, double spacing, RotationDirection dir)
        {
            Sliced_path_item spiral = new Sliced_path_item(Sliced_path_item_type.SPIRAL);
            foreach (Biarc2d biarc in Spiral_generator.Gen_archimedean_spiral(start, end, spacing, dir))
                spiral.Add(biarc, _general_tolerance);
            Path.Add(spiral);
        }

        public void Append_root_slice(Slice slice)
        {
            append_slice(slice);
        }        

        public virtual void Append_slice(Slice slice, List<Point2F> guide)
        {
            if (_last_slice == null)
                throw new Exception("attempt to install slice without the root slice");

            if (guide == null)
            {
                Path.Add(connect_slices(slice, _last_slice));
            }
            else
            {
                Sliced_path_item p = new Sliced_path_item(Sliced_path_item_type.GUIDE);

                p.Add(_last_slice.End);
                foreach (Point2F pt in guide)
                    p.Add(pt);
                p.Add(slice.Start);

                Path.Add(p);
            }
            
            append_slice(slice);
        }

        public void Append_return_to_base(List<Point2F> exit)
        {
            Sliced_path_item p = new Sliced_path_item(Sliced_path_item_type.RETURN_TO_BASE);

            p.Add(_last_slice.End);
            foreach (Point2F pt in exit)
                p.Add(pt);

            Path.Add(p);
        }

        public Sliced_path_generator(double general_tolerance)
        {
            _general_tolerance = general_tolerance;
        }
    }

    class Sliced_path_smooth_generator : Sliced_path_generator
    {
        double _min_arc_len;

        private bool is_biarc_inside_ball(Biarc2d biarc, Circle2F ball)
        {
            if (biarc.Pm.DistanceTo(ball.Center) > ball.Radius)
                return false;

            Point2F start = biarc.P1;
            Point2F end = biarc.P2;
            Line2F insects;

            if (biarc.Seg1 is Line2F)
                insects = ball.LineIntersect((Line2F)biarc.Seg1);
            else
                insects = ((Arc2F)biarc.Seg1).CircleIntersect(ball);

            if ((!insects.p1.IsUndefined) && insects.p1.DistanceTo(start) < _general_tolerance)
                insects.p1 = Point2F.Undefined;
            if ((!insects.p2.IsUndefined) && insects.p2.DistanceTo(start) < _general_tolerance)
                insects.p2 = Point2F.Undefined;

            if (!(insects.p1.IsUndefined && insects.p2.IsUndefined))
                return false;

            if (biarc.Seg2 is Line2F)
                insects = ball.LineIntersect((Line2F)biarc.Seg2);
            else
                insects = ((Arc2F)biarc.Seg2).CircleIntersect(ball);

            if ((!insects.p1.IsUndefined) && insects.p1.DistanceTo(end) < _general_tolerance)
                insects.p1 = Point2F.Undefined;
            if ((!insects.p2.IsUndefined) && insects.p2.DistanceTo(end) < _general_tolerance)
                insects.p2 = Point2F.Undefined;

            if (!(insects.p1.IsUndefined && insects.p2.IsUndefined))
                return false;

            return true;
        }

        private Sliced_path_item connect_slices_with_biarc(Slice dst, Slice src)
        {
            Point2F start = src.End;
            Point2F end = dst.Start;
            // unit normals to points
            Vector2d vn_start = new Vector2d(src.Center, start).Unit();
            Vector2d vn_end = new Vector2d(dst.Center, end).Unit();
            // tangents to points
            Vector2d vt_start;
            Vector2d vt_end;
            if (src.Dir == RotationDirection.CW)
                vt_start = new Vector2d(vn_start.Y, -vn_start.X);
            else
                vt_start = new Vector2d(-vn_start.Y, vn_start.X);

            if (dst.Dir == RotationDirection.CW)
                vt_end = new Vector2d(vn_end.Y, -vn_end.X);
            else
                vt_end = new Vector2d(-vn_end.Y, vn_end.X);

            Biarc2d biarc = new Biarc2d(start, vt_start, end, vt_end);

            if (!is_biarc_inside_ball(biarc, src.Ball))
                return null;

            Sliced_path_item path = new Sliced_path_item(Sliced_path_item_type.SMOOTH_CHORD);
            path.Add(biarc, _general_tolerance);
            return path;
        }

        private Sliced_path_item smooth_connect_slices(Slice dst, Slice src)
        {
            Sliced_path_item path = null;

            // do not emit biarcs if distance is too small
            if (src.End.DistanceTo(dst.Start) > _min_arc_len)
            {
                path = connect_slices_with_biarc(dst, src);
                if (path == null)
                    Logger.warn("biarc is outside the slice, replacing with chord");
            }
            return path;
        }

        public override void Append_slice(Slice slice, List<Point2F> guide)
        {
            if (guide == null)
            {
                Sliced_path_item biarc = smooth_connect_slices(slice, _last_slice);
                if (biarc != null)
                {
                    Path.Add(biarc);
                    append_slice(slice);
                    return;
                }
            }

            base.Append_slice(slice, guide);            
        }

        public Sliced_path_smooth_generator(double general_tolerance, double min_arc_len) : base(general_tolerance)
        {
            _min_arc_len = min_arc_len;
        }
    }
}
