using System;
using System.Collections.Generic;

using CamBam.Geom;

namespace Matmill
{
    // special structure for a fast joining of chained line segments
    // segments (references to them) are stored in dictionary under the long integer key, crafted from
    // their coordinates. segment is stored twice, under the keys for start and end points.
    // lookup should give a small range of nearby segments, then finepicked by the distance compare
    // this way we may find next segment(s) in chain in almost O(1)

    // pull operation removes chained segments from pool and return them.

    // segments may map to different keys if their coordinates are rounded to neighbour integers,
    // this is remedied by storing 4 hashes instead of one, with x and y coordinates floored and
    // ceiled. it corresponds to 4 nearby cells in 2d grid
    class Segpool
    {
        private Dictionary<ulong, List<Line2F>> _pool;
        private readonly double _tolerance;

        public int N_hashes { get { return _pool.Count; } }

        private ulong[] hash(Point2F pt)
        {
            double hashscale = 1 / (_tolerance * 10);
            double x = pt.X * hashscale;
            double y = pt.Y * hashscale;

            return new ulong[]
            {
                    ((ulong)Math.Floor(y) << 32) | (ulong)Math.Floor(x),
                    ((ulong)Math.Floor(y) << 32) | (ulong)Math.Ceiling(x),
                    ((ulong)Math.Ceiling(y) << 32) | (ulong)Math.Floor(x),
                    ((ulong)Math.Ceiling(y) << 32) | (ulong)Math.Ceiling(x),
            };
        }

        private void insert_seg(ulong h, Line2F seg)
        {
            if (!_pool.ContainsKey(h))
                _pool[h] = new List<Line2F>();
            if (!_pool[h].Contains(seg))
                _pool[h].Add(seg);
        }

        private void remove_seg(ulong h, Line2F seg)
        {
            if (_pool.ContainsKey(h))
                _pool[h].Remove(seg);
        }

        // add line hashed by the beginning or end point
        public void Add(Line2F seg, bool reverse)
        {
            ulong[] h = hash(reverse ? seg.p2 : seg.p1);

            insert_seg(h[0], seg);
            insert_seg(h[1], seg);
            insert_seg(h[2], seg);
            insert_seg(h[3], seg);
        }

        public void Remove(Line2F seg)
        {
            ulong[] h1 = hash(seg.p1);

            remove_seg(h1[0], seg);
            remove_seg(h1[1], seg);
            remove_seg(h1[2], seg);
            remove_seg(h1[3], seg);

            ulong[] h2 = hash(seg.p2);

            remove_seg(h2[0], seg);
            remove_seg(h2[1], seg);
            remove_seg(h2[2], seg);
            remove_seg(h2[3], seg);
        }

        public List<Point2F> Pull_follow_points(Point2F join_pt)
        {
            List<Point2F> followers = new List<Point2F>();
            List<Line2F> processed = new List<Line2F>();

            ulong[] h = hash(join_pt);

            for (int i = 0; i < 4; i++)
            {
                if (!_pool.ContainsKey(h[i]))
                    continue;

                foreach (Line2F seg in _pool[h[i]])
                {
                    if (processed.Contains(seg))
                        continue;  // already got it

                    if (join_pt.DistanceTo(seg.p1) < _tolerance)
                    {
                        followers.Add(seg.p2);
                        processed.Add(seg);
                    }
                    else if (join_pt.DistanceTo(seg.p2) < _tolerance)
                    {
                        followers.Add(seg.p1);
                        processed.Add(seg);
                    }
                }
            }

            foreach (Line2F seg in processed)
            {
                Remove(seg);
            }

            return followers;
        }

        public bool Contains_point(Point2F pt)
        {
            ulong[] h = hash(pt);

            for (int i = 0; i < 4; i++)
            {
                if (!_pool.ContainsKey(h[i]))
                    continue;

                foreach (Line2F seg in _pool[h[i]])
                {
                    if (pt.DistanceTo(seg.p1) < _tolerance)
                        return true;
                    if (pt.DistanceTo(seg.p2) < _tolerance)
                        return true;
                }
            }
            return false;
        }

        public Segpool(int capacity, double tolerance)
        {
            _pool = new Dictionary<ulong, List<Line2F>>(capacity * 2 * 4);
            _tolerance = tolerance;
        }
    }
}
