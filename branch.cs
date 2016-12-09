using System;
using System.Collections;
using System.Collections.Generic;

using CamBam.CAD;
using CamBam.Geom;

namespace Matmill
{
    class Branch : Medial_branch
    {
        private readonly Curve _curve = new Curve();
        private readonly List<Branch> _children = new List<Branch>();
        private readonly Branch Parent = null;

        //--- Medial_branch interface

        public double Deep_distance                 { get { return calc_deep_distance();  } }
        public double Shallow_distance              { get { return _curve.Length; } }
        public Point2F Start                        { get { return _curve.Start; } }
        public Point2F End                          { get { return _curve.End; } }

        public void Add_point(Point2F pt)
        {
            _curve.Add(pt);
        }

        public Medial_branch Spawn_child()
        {
            return new Branch(this);
        }

        public void Postprocess()
        {
            // sort children so the shortest branch comes first in df traverse
            this._children.Sort((a, b) => a.Deep_distance.CompareTo(b.Deep_distance));
        }

        public void Attach_to_parent()
        {
            if (this.Parent == null)
                throw new Exception("attempt to attach orphaned branch");
            this.Parent._children.Add(this);
        }

        //--- own interface

        public delegate int Branch_visitor(Point2F pt);
        public readonly List<Slice> Slices = new List<Slice>();

        public Pocket_path_item Entry;

        public bool Is_leaf { get { return _children.Count == 0; } }

        public List<Branch> Df_traverse()  //
        {
            List<Branch> result = new List<Branch>();
            result.Add(this);
            foreach (Branch b in _children)
                result.AddRange(b.Df_traverse());
            return result;
        }

        public double calc_deep_distance()
        {
            double dist = _curve.Length;
            foreach (Branch b in _children)
                dist += b.Deep_distance;
            return dist;
        }

        public Slice Get_upstream_slice()
        {
            Branch b;

            for (b = this; b != null && b.Slices.Count == 0; b = b.Parent);

            if (b == null) return null;
            return b.Slices[b.Slices.Count - 1];
        }

        public void Bisect(Branch_visitor visitor, ref double t, double stop_distance)
        {
            if (t < 0.0 || t > 1.0)
                throw new Exception("branch bisector was called with a wrong range");

            double left = t;
            double right = 1.0;

            double mid;

            while (true)
            {
                mid = (left + right) / 2;
                Point2F pt = _curve.Get_parametric_pt(mid);

                int result = visitor(pt);

                if (result == 0)
                    break;

                if (result < 0)
                    right = mid;
                else if (result > 0)
                    left = mid;

                Point2F other = _curve.Get_parametric_pt(left == mid ? right : left);
                if (pt.DistanceTo(other) < stop_distance)   // range has shrinked, stop search
                    break;
            }

            t = mid;
        }

        public Polyline To_polyline()
        {
            return _curve.To_polyline();
        }

        public Branch(Branch parent)
        {
            Parent = parent;
        }
    }
}