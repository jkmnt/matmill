using System.Collections.Generic;

using CamBam.CAD;

namespace Medial_demo
{
    class Branch
    {
        public readonly Polyline Curve = new Polyline();
        public readonly Branch Parent = null;
        public readonly List<Branch> Children = new List<Branch>();

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
            double dist = Curve.GetPerimeter();
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

        public Branch(Branch parent)
        {
            Parent = parent;
        }
    }

}