using System;
using System.Collections;
using System.Collections.Generic;

using CamBam.Geom;

namespace Matmill
{    
    class Branch
    {
        public delegate int Branch_visitor(Point2F pt);

        public readonly Curve Curve = new Curve();
        public readonly Branch Parent = null;
        public readonly List<Branch> Children = new List<Branch>();
        public readonly List<Slice> Slices = new List<Slice>();
        public Pocket_path_item Entry = null;

        public bool Is_leaf { get { return Children.Count == 0; } }

        public List<Branch> Df_traverse()  //
        {
            List<Branch> result = new List<Branch>();
            result.Add(this);
            foreach (Branch b in Children)
                result.AddRange(b.Df_traverse());
            return result;
        }

        public double Deep_distance()
        {
            double dist = Curve.Length;
            foreach (Branch b in Children)
                dist += b.Deep_distance();

            return dist;
        }

        // Get all the slices blocking path (meet first) while traveling up the branch
        // (and followind neighbour downstream subbranches)
        public List<Slice> Get_upstream_roadblocks()
        {
            List<Slice> candidates = new List<Slice>();

            for (Branch visited = this; visited.Parent != null; visited = visited.Parent)
            {
                Branch upstream = visited.Parent;

                if (upstream.Children.Count != 0)
                {
                    foreach (Branch child in upstream.Children)
                    {
                        // except the path we're walking now
                        if (child != visited)
                            candidates.AddRange(child.Get_downstream_roadblocks());
                    }
                }

                if (upstream.Slices.Count != 0)
                {
                    candidates.Add(upstream.Slices[upstream.Slices.Count - 1]);
                    break;
                }
            }

            return candidates;
        }

        public Slice Get_upstream_slice()
        {
            Branch b;

            for (b = this; b != null && b.Slices.Count == 0; b = b.Parent);

            if (b == null) return null;
            return b.Slices[b.Slices.Count - 1];
        }

        // Get all the slices blocking path (meet first) while traveling down the branch
        // and next subbranches
        public List<Slice> Get_downstream_roadblocks()
        {
            List<Slice> candidates = new List<Slice>();

            if (Slices.Count != 0)
            {
                candidates.Add(Slices[0]);
            }
            else
            {
                foreach (Branch c in Children)
                    candidates.AddRange(c.Get_downstream_roadblocks());
            }

            return candidates;
        }

        public Branch(Branch parent)
        {
            Parent = parent;
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
                Point2F pt = Curve.Get_parametric_pt(mid);

                int result = visitor(pt);

                if (result == 0)
                    break;

                if (result < 0)
                    right = mid;
                else if (result > 0)
                    left = mid;

                Point2F other = Curve.Get_parametric_pt(left == mid ? right : left);
                if (pt.DistanceTo(other) < stop_distance)   // range has shrinked, stop search
                    break;
            }

            t = mid;
        }
    }
}