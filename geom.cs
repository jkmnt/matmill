using System;
using System.Collections.Generic;

using CamBam.Geom;
using CamBam.CAD;

namespace Geom
{
    public struct Vector2d
    {
        public double X;
        public double Y;

        public double Mag
        {
            get { return Math.Sqrt(X * X + Y * Y); }
        }

        public double Angle
        {
            get { return Math.Atan2(Y, X); }
        }

        public double Ccw_angle
        {
            get
            {
                double angle = this.Angle;
                return angle >= 0 ? angle : angle + 2.0 * Math.PI;
            }
        }

        public Point2F Point
        {
            get { return (Point2F)this; }
        }

        public Vector2d(double x, double y)
        {
            X = x;
            Y = y;
        }

        public Vector2d(Point2F pt)
        {
            X = pt.X;
            Y = pt.Y;
        }

        public Vector2d(Point2F start, Point2F end)
        {
            X = end.X - start.X;
            Y = end.Y - start.Y;
        }

        public Vector2d(Vector2d v)
        {
            X = v.X;
            Y = v.Y;
        }

        public Vector2d Normal()
        {
            return new Vector2d(-Y, X);
        }

        public Vector2d Inverted()
        {
            return new Vector2d(-X, -Y);
        }

        public Vector2d Unit()
        {
            double mag = Mag;
            return new Vector2d(X / mag, Y / mag);
        }

        public Vector2d Rotated(double theta)
        {
            double x = X * Math.Cos(theta) - Y * Math.Sin(theta);
            double y = X * Math.Sin(theta) + Y * Math.Cos(theta);

            return new Vector2d(x, y);
        }

        public double Det(Vector2d b)
        {
            return X * b.Y - Y * b.X;
        }

        public double Angle_to(Vector2d b)
        {
            return Math.Atan2(this.Det(b), this * b);
        }

        public double Ccw_angle_to(Vector2d b)
        {
            double angle = this.Angle_to(b);
            return angle >= 0 ? angle : angle + 2.0 * Math.PI;
        }

        public static explicit operator Point2F(Vector2d v)
        {
            return new Point2F(v.X, v.Y);
        }

        public static double operator *(Vector2d a, Vector2d b)
        {
            return a.X * b.X + a.Y * b.Y;
        }

        public static Vector2d operator *(Vector2d a, double d)
        {
            return new Vector2d(a.X * d, a.Y * d);
        }

        public static Vector2d operator *(double d, Vector2d a)
        {
            return a * d;
        }

        public static Vector2d operator +(Vector2d a, Vector2d b)
        {
            return new Vector2d(a.X + b.X, a.Y + b.Y);
        }

        public static Vector2d operator -(Vector2d a, Vector2d b)
        {
            return new Vector2d(a.X - b.X, a.Y - b.Y);
        }
    }

    public struct Biarc2d
    {
        public object Seg1;
        public object Seg2;

        public Point2F P1
        {
            get
            {
                return this.Seg1 is Arc2F ? ((Arc2F)this.Seg1).P1 : ((Line2F)this.Seg1).p1;
            }
        }

        public Point2F Pm
        {
            get
            {
                return this.Seg1 is Arc2F ? ((Arc2F)this.Seg1).P2 : ((Line2F)this.Seg1).p2;
            }
        }

        public Point2F P2
        {
            get
            {
                return this.Seg2 is Arc2F ? ((Arc2F)this.Seg2).P2 : ((Line2F)this.Seg2).p2;
            }
        }

        /* Adapted from http://www.ryanjuckett.com/programming/biarc-interpolation
        */
        private static Point2F calc_pm(Point2F p1, Vector2d t1, Point2F p2, Vector2d t2)
        {
            Vector2d v = new Vector2d(p2 - p1);
            Vector2d t = t1 + t2;
            double v_dot_t = v * t;
            double t1_dot_t2 = t1 * t2;

            double d2;
            double d2_denom = 2 * (1 - t1_dot_t2);

            if (d2_denom == 0)  // equal tangents
            {
                d2_denom = 4.0 * (v * t2);

                d2 = v * v / d2_denom;

                if (d2_denom == 0)  // v perpendicular to tangents
                    return p1 + (v * 0.5).Point;
            }
            else    // normal case
            {
                double d2_num = -v_dot_t + Math.Sqrt(v_dot_t * v_dot_t + 2 * (1 - t1_dot_t2) * (v * v));
                d2 = d2_num / d2_denom;
            }

            return (p1 + p2 + (d2 * (t1 - t2)).Point) * 0.5;
        }

        private static object[] calc_arcs(Point2F p1, Vector2d t1, Point2F p2, Vector2d t2, Point2F pm)
        {
            object[] segs = new object[2];

            Vector2d n1 = t1.Normal();

            Vector2d pm_minus_p1 = new Vector2d(pm - p1);
            double c1_denom = 2 * (n1 * pm_minus_p1);
            if (c1_denom == 0)  // c1 is at infinity, replace arc with line
            {
                segs[0] = new Line2F(p1, pm);
            }
            else
            {
                Point2F c1 = p1 + (pm_minus_p1 * pm_minus_p1 / c1_denom * n1).Point;
                RotationDirection dir1 = new Vector2d(p1 - c1) * n1 > 0 ? RotationDirection.CW : RotationDirection.CCW;
                segs[0] = new Arc2F(c1, p1, pm, dir1);
            }

            Vector2d n2 = t2.Normal();
            Vector2d pm_minus_p2 = new Vector2d(pm - p2);
            double c2_denom = 2 * (n2 * pm_minus_p2);
            if (c2_denom == 0)  // c2 is at infinity, replace arc with line
            {
                segs[1] = new Line2F(pm, p2);
            }
            else
            {
                Point2F c2 = p2 + (pm_minus_p2 * pm_minus_p2 / c2_denom * n2).Point;
                RotationDirection dir2 = new Vector2d(p2 - c2) * n2 > 0 ? RotationDirection.CW : RotationDirection.CCW;
                segs[1] = new Arc2F(c2, pm, p2, dir2);
            }

            return segs;
        }

        public Polyline To_polyline(double tolerance)
        {
            Polyline p = new Polyline();

            if (Seg1 is Arc2F)
                p.Add((Arc2F)Seg1, tolerance);
            else
                p.Add((Line2F)Seg1, tolerance);

            if (Seg2 is Arc2F)
                p.Add((Arc2F)Seg2, tolerance);
            else
                p.Add((Line2F)Seg2, tolerance);

            return p;
        }

        public Biarc2d(Point2F p1, Vector2d t1, Point2F p2, Vector2d t2)
        {
            Point2F pm = calc_pm(p1, t1, p2, t2);
            object[] segs = calc_arcs(p1, t1, p2, t2, pm);
            this.Seg1 = segs[0];
            this.Seg2 = segs[1];
        }
    }

    public class Curve2d
    {
        // XXX: empty curve shouldn't be accessed
        private List<Point2F> _points = new List<Point2F>();
        private List<double> _seg_offsets = new List<double>();
        private double _len = 0;

        public List<Point2F> Points { get { return _points; } }
        public Point2F Start { get { return _points[0]; } }
        public Point2F End { get { return _points[_points.Count - 1]; } }
        public double Length { get { return _len; } }

        public void Add(Point2F pt)
        {
            if (_points.Count != 0)
            {
                double seg_len = End.DistanceTo(pt);
                _seg_offsets.Add(_len);
                _len += seg_len;
            }
            _points.Add(pt);
        }

        public Point2F Get_parametric_pt(double u)
        {
            if (_points.Count < 2)
                return _points[0];

            double offset = u * _len;

            int seg = _seg_offsets.BinarySearch(offset);

            // if there is no exact element found, binary search returns 1's complemented index of the
            // first larger element, so we use it to find last smaller element
            if (seg < 0)
                seg = ~seg - 1;

            offset -= _seg_offsets[seg];

            Point2F p1 = _points[seg];
            Point2F p2 = _points[seg + 1];
            double dist = p2.DistanceTo(p1);
            double x = p1.X + offset / dist * (p2.X - p1.X);
            double y = p1.Y + offset / dist * (p2.Y - p1.Y);

            return new Point2F(x, y);
        }

        public Polyline To_polyline()
        {
            Polyline p = new Polyline();
            foreach (Point2F pt in Points)
                p.Add((Point3F)pt);
            return p;
        }
    }
}