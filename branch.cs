using System;
using System.Collections;
using System.Collections.Generic;

namespace Matmill
{
    class Branch
    {
        public delegate void Visitor(Branch b);

        public readonly Curve Curve = new Curve();
        public readonly Branch Parent = null;
        public readonly List<Branch> Children = new List<Branch>();
        public readonly List<Slice> Slices = new List<Slice>();
        public object Entry = null;
        public string Debug = "";

        public bool Is_leaf { get { return Children.Count == 0; } }

        public List<Branch> Df_traverse()  //
        {
            List<Branch> result = new List<Branch>();
            result.Add(this);
            foreach (Branch b in Children)
                result.AddRange(b.Df_traverse());
            return result;
        }

        public void Df_visit(Visitor v)
        {
            v(this);
            foreach (Branch b in Children)
                b.Df_visit(v);
        }

        public double Deep_distance()
        {
            double dist = Curve.Length;
            foreach (Branch b in Children)
                dist += b.Deep_distance();

            return dist;
        }

        public List<Branch> Get_parents()
        {
            List<Branch> parents = new List<Branch>();
            for (Branch p = Parent; p != null; p = p.Parent)
            {
                parents.Add(p);
            }
            return parents;
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
    }

}