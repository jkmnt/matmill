using System;
using System.Collections.Generic;

using CamBam.Geom;

namespace Matmill
{
    class Curve
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
            // TODO: use binary search
            int seg;
            for (seg = 1; seg < _seg_offsets.Count; seg++)
            {
                if (_seg_offsets[seg] > offset)
                    break;
            }

            seg -= 1;
            offset -= _seg_offsets[seg];

            Point2F p1 = _points[seg];
            Point2F p2 = _points[seg + 1];
            double dist = p2.DistanceTo(p1);
            double x = p1.X + offset / dist * (p2.X - p1.X);
            double y = p1.Y + offset / dist * (p2.Y - p1.Y);

            return new Point2F(x, y);
        }

        public Curve()
        {
        }
    }
}