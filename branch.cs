using System;
using System.Collections;
using System.Collections.Generic;

using CamBam.CAD;
using CamBam.Geom;

namespace Matmill
{

    delegate int Branch_visitor(Point2F pt);

    interface Branch
    {
        void Bisect(Branch_visitor visitor, ref double t, double stop_distance);
        IEnumerable Children { get; }
        Point2F Start { get; }
        Point2F End { get; }
    }

    class Pocket_branch : Medial_branch, Branch
    {
        private readonly Curve _curve = new Curve();
        private readonly Pocket_branch Parent = null;
        private readonly List<Pocket_branch> _children = new List<Pocket_branch>();

        //--- Medial_branch interface

        public double Deep_distance { get { return calc_deep_distance(); } }
        public double Shallow_distance { get { return _curve.Length; } }
        public Point2F Start { get { return _curve.Start; } }
        public Point2F End { get { return _curve.End; } }

        public void Add_point(Point2F pt)
        {
            _curve.Add(pt);
        }

        public Medial_branch Spawn_child()
        {
            return new Pocket_branch(this);
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
        public bool Is_leaf { get { return _children.Count == 0; } }

        public IEnumerable Children
        {
            get { return _children; }
        }

        public List<Pocket_branch> Df_traverse()  //
        {
            List<Pocket_branch> result = new List<Pocket_branch>();
            result.Add(this);
            foreach (Pocket_branch b in _children)
                result.AddRange(b.Df_traverse());
            return result;
        }

        public double calc_deep_distance()
        {
            double dist = _curve.Length;
            foreach (Pocket_branch b in _children)
                dist += b.Deep_distance;
            return dist;
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

        public Pocket_branch(Pocket_branch parent)
        {
            Parent = parent;
        }
    }

}