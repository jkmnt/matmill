using System;
using System.Collections;
using System.Collections.Generic;

using CamBam.UI;
using CamBam.CAD;
using CamBam.Geom;

using Voronoi2;

namespace Matmill
{
    public class Pocket_generator
    {
        class Sweep_comparer : IComparer
        {
            Point2F _origin;
            Point2F _center;
            RotationDirection _dir;
            Vector2F _start_vector;
            public Sweep_comparer(Point2F origin, Point2F center, RotationDirection dir)
            {
                _origin = origin;
                _center = center;
                _dir = dir;
                _start_vector = new Vector2F(center, origin);
            }

            double angle_to_start_vector(Point2F p)
            {
                Vector2F v = new Vector2F(_center, p);
                double angle = Math.Atan2(Vector2F.Determinant(_start_vector, v), Vector2F.DotProduct(_start_vector, v));
                return (_dir == RotationDirection.CCW) ? angle : (2.0 * Math.PI - angle);
            }

            public int Compare(object a, object b)
            {
                double d0 = angle_to_start_vector((Point2F)a);
                double d1 = angle_to_start_vector((Point2F)b);

                if (d0 < d1) return -1;
                if (d0 > d1) return 1;
                return 0;
            }
        }

        class Slice
        {
            Circle2F _ball;
            Point2F _p1;
            Point2F _p2;
            List<Arc2F> _segments = new List<Arc2F>();
            double _max_engagement;

            public Circle2F Ball { get { return _ball; } }

            public Point2F Center { get { return _ball.Center; }}
            public double Radius { get { return _ball.Radius; }}
            public double Max_engagement { get { return _max_engagement; }}
            public List<Arc2F> Segments { get { return _segments; } }

            static public double Calc_max_engagement(Point2F center, double radius, Slice prev_slice)
            {
                double delta_s = Point2F.Distance(center, prev_slice.Center);
                double delta_r = radius - prev_slice.Radius;
                return delta_s + delta_r;
            }

            public Slice(Point2F center, double radius, Slice prev_slice, RotationDirection dir)
            {
                _max_engagement = Slice.Calc_max_engagement(center, radius, prev_slice);

                Line2F insects = prev_slice.Ball.CircleIntersect(new Circle2F(center, radius));

                if (insects.p1.IsUndefined || insects.p2.IsUndefined)
                {
                    ;
                }
                else
                {
                    _ball = new Circle2F(center, radius);

                    Arc2F arc = new Arc2F(center, insects.p1, insects.p2, dir);

                    if (! arc.VectorInsideArc(new Vector2F(prev_slice.Center, center)))
                        arc = new Arc2F(center, insects.p2, insects.p1, dir);

                    _p1 = arc.P1;
                    _p2 = arc.P2;

                    _segments.Add(arc);
                }
            }

            public Slice(Point2F center, double radius, RotationDirection dir)
            {
                _ball = new Circle2F(center, radius);
                _max_engagement = 0;
                // XXX: hack, just for now
                Arc2F arc0 = new Arc2F(center, radius, 0, 120);
                Arc2F arc1 = new Arc2F(center, radius, 120, 120);
                Arc2F arc2 = new Arc2F(center, radius, 240, 120);
                _segments.Add(arc0);
                _segments.Add(arc1);
                _segments.Add(arc2);
            }
        }

        class Branch
        {
            public Polyline Curve = null;
            public Branch Parent = null;
            public List<Branch> Children = new List<Branch>();
            public bool End_is_leaf = true;
            public List<Slice> Slices = new List<Slice>();

            public List<Branch> Df_traverse()  //
            {
                List<Branch> result = new List<Branch>();
                result.Add(this);
                foreach (Branch b in Children)
                    result.AddRange(b.Df_traverse());
                return result;
            }

            /*
            public List<Branch> Df_traverse()  //
            {
                List<Branch> result = new List<Branch>();
                result.Add(this);
                foreach (Branch b in Children)
                    result.AddRange(b.Bf_traverse());
                return result;
            }
            */

            public double Deep_distance()
            {
                double dist = Curve.GetPerimeter();
                foreach (Branch b in Children)
                    dist += b.Deep_distance();
                return dist;
            }
        }

        double GENERAL_TOLERANCE = 0.001;
        double VORONOI_MARGIN = 1.0;

        Region _reg;
        double _cutter_r = 3.0;
        double _max_engagement = 3.0 * 0.4;
        double _sample_distance = 0.1;

        public double cutter_d        {set { _cutter_r = value / 2.0;}}
        public double max_engagement  {set { _max_engagement = value; } }
        public double sample_distance {set { _sample_distance = value; } }

        Point3F point(Point2F p2)
        {
            return new Point3F(p2.X, p2.Y, 0);
        }

        Point2F point(Point3F p3)
        {
            return new Point2F(p3.X, p3.Y);
        }

        void attach_branches(Branch me, List<Polyline> pool)
        {
            // TODO: maybe the pool could be modified while traversal and to_remove is not needed ?
            List<Polyline> to_remove = new List<Polyline>();

            foreach (Polyline p in pool)
            {
                if (p == me.Curve)
                    continue;

                if (p.Points.Count < 1)
                {
                    Host.log("empty polyline detected. Strange");
                    continue;
                }

                Vector2F normal = new Vector2F();
                int seg = 0;

                Point2F pt_start = point(p.Points[0].Point);
                Point2F pt_end = point(p.Points[p.Points.Count - 1].Point);

                Point2F pt = point(me.Curve.GetNearestPoint(pt_start, ref normal, ref seg, true));
                if (Point2F.Distance(pt_start, pt) < GENERAL_TOLERANCE)
                {
                    ;
                }
                else
                {
                    pt = point(me.Curve.GetNearestPoint(pt_end, ref normal, ref seg, true));
                    if (Point2F.Distance(pt_end, pt) < GENERAL_TOLERANCE)
                    {
                        p.Reverse();
                    }
                    else
                    {
                        continue;
                    }
                }

                Branch b = new Branch();
                b.Curve = p;
                b.Parent = me;
                me.Children.Add(b);
                to_remove.Add(p);
            }

            Point2F my_end = point(me.Curve.Points[me.Curve.Points.Count - 1].Point);
            List<Branch> followers = new List<Branch>();

            foreach(Branch b in me.Children)
            {
                Point2F his_start = point(b.Curve.Points[0].Point);
                if (Point2F.Distance(my_end, his_start) < GENERAL_TOLERANCE)
                    followers.Add(b);
            }
            if (followers.Count > 0)
                me.End_is_leaf = false;

            foreach (Polyline p in to_remove)
                pool.Remove(p);

            Host.log("end is leaf {0}, childcount {1}", me.End_is_leaf, me.Children.Count);

            if (pool.Count != 0)
            {
                foreach(Branch b in me.Children)
                    attach_branches(b, pool);
            }

            // try to attach next longest branch to me if possible
            //XXX: maybe it's a wrong thing. Will decide later
            //if (true)
            // {
            if (me.Children.Count == 1)
            {
                if (followers.Count != 0)
                {
                    Host.log("joining follower");

                    Branch longest_follower = null;
                    double max_dist = double.MinValue;
                    foreach (Branch f in followers)
                    {
                        double dist = f.Deep_distance();
                        if (dist > max_dist)
                        {
                            max_dist = dist;
                            longest_follower = f;
                        }
                    }

                    me.Children.Remove(longest_follower);
                    Polyline follower_poly = longest_follower.Curve;

                    // XXX: bulge may be wrong
                    for (int i = 1; i < follower_poly.Points.Count; i++)
                    {
                        me.Curve.Add(follower_poly.Points[i].Point, follower_poly.Points[i].Bulge);
                    }
                    me.Children.AddRange(longest_follower.Children);
                    me.End_is_leaf = longest_follower.End_is_leaf;
                    foreach (Branch b in me.Children)
                        b.Parent = me;
                }
            }

            me.Curve.Closed = false;    // be on a safe side
        }

        Branch get_branches(Polyline[] polys, double radius_sample_step)
        {
            Point2F max_radius_pt = Point2F.Undefined;
            Polyline root_poly = new Polyline();
            double max_radius = 0;
            // find the branch with the roughly biggest radius
            foreach (Polyline p in polys)
            {
                List<Point2F> samples = sample_curve(p, radius_sample_step);
                foreach (Point2F pt in samples)
                {
                    double rad = get_mic_radius(pt);
                    if (rad > max_radius)
                    {
                        max_radius = rad;
                        root_poly = p;
                        max_radius_pt = pt;
                    }
                }
            }

            List<Polyline> pool = new List<Polyline>(polys);

            // split it there if requried
            // XXX: make sure there is a points ?
            if (    Point2F.Distance(max_radius_pt, point(root_poly.Points[0].Point)) > radius_sample_step * 2
                &&  Point2F.Distance(max_radius_pt, point(root_poly.Points[root_poly.Points.Count - 1].Point)) > radius_sample_step * 2)
            {
                Polyline[] splitted = root_poly.SplitAtPoint(max_radius_pt, GENERAL_TOLERANCE);
                if (splitted.Length > 1)
                {
                    pool.Remove(root_poly);
                    pool.AddRange(splitted);
                    root_poly = splitted[0];
                    if (Point2F.Distance(point(root_poly.Points[0].Point), max_radius_pt) > GENERAL_TOLERANCE)
                        root_poly.Reverse();
                }
            }

            Branch root = new Branch();
            root.Curve = root_poly;
            pool.Remove(root_poly);
            attach_branches(root, pool);
            return root;
        }

        List<Point2F> sample_curve(Polyline p, double step)
        {
            List<Point2F> points = new List<Point2F>();
            foreach (Point3F pt in PointListUtils.CreatePointlistFromPolylineStep(p, step).Points.ToArray())
                points.Add(new Point2F(pt.X, pt.Y));
            if (points.Count < 2) return points;
            // sometimes first and last points would be too close to each other for the closed shapes, remove dupe
            if (Point2F.Distance(points[0], points[points.Count - 1]) < GENERAL_TOLERANCE)
            {
                Host.log("removing duplicate point from pointlist");
                points.RemoveAt(points.Count - 1);
            }
            return points;
        }

        List<Polyline> get_mat_segments()
        {
            List<Point2F> plist = new List<Point2F>();

            plist.AddRange(sample_curve(this._reg.OuterCurve, _max_engagement));
            foreach (Polyline p in this._reg.HoleCurves)
                plist.AddRange(sample_curve(p, _max_engagement));

            Host.log("Got {0} points", plist.Count);

            double[] xs = new double[plist.Count];
            double[] ys = new double[plist.Count];

            double min_x = double.MaxValue;
            double max_x = double.MinValue;
            double min_y = double.MaxValue;
            double max_y = double.MinValue;

            for (int i = 0; i < plist.Count; i++)
            {
                xs[i] = plist[i].X;
                ys[i] = plist[i].Y;
                if (xs[i] < min_x) min_x = xs[i];
                if (xs[i] > max_x) max_x = xs[i];
                if (ys[i] < min_y) min_y = ys[i];
                if (ys[i] > max_y) max_y = ys[i];
            }

            min_x -= VORONOI_MARGIN;
            max_x += VORONOI_MARGIN;
            min_y -= VORONOI_MARGIN;
            max_y += VORONOI_MARGIN;

            List<GraphEdge> edges = new Voronoi(GENERAL_TOLERANCE).generateVoronoi(xs, ys, min_x, max_x, min_y, max_y);

            Host.log("voroning completed");

            List<Polyline> inner_segments = new List<Polyline>();

            foreach (GraphEdge e in edges)
            {
                Polyline poly = new Line(e.x1, e.y1, e.x2, e.y2).ToPolyline();

                if (this._reg.OuterCurve.Intersects(poly)) continue;
                if (this._reg.OuterCurve.PolylineOutsidePolyline(poly, GENERAL_TOLERANCE)) continue;

                bool should_add = true;
                foreach (Polyline hole in this._reg.HoleCurves)
                {
                    if (hole.Intersects(poly) || hole.PolylineInsidePolyline(poly, GENERAL_TOLERANCE))
                    {
                        should_add = false;
                        break;
                    }
                }

                if (should_add)
                    inner_segments.Add(poly);
            }

            return inner_segments;
        }

        double get_mic_radius(Point2F pt)
        {
            double radius;

            Vector2F normal = new Vector2F();
            int seg = 0;

            Point3F nearest = this._reg.OuterCurve.GetNearestPoint(pt, ref normal, ref seg, true);
            radius = Point2F.Distance(point(nearest), pt);

            foreach (Polyline hole in this._reg.HoleCurves)
            {
                nearest = hole.GetNearestPoint(pt, ref normal, ref seg, true);
                double dist = Point2F.Distance(point(nearest), pt);
                if (dist < radius)
                    radius = dist;
            }

            // account for margin just in one subrtract. Nice !
            if (false)
            {
                radius -= _cutter_r;
            }

            return radius ;
        }

        Slice find_nearest_slice(Branch b, Point2F center)
        {
            while (b.Slices.Count == 0 && b.Parent != null)
            {
                Host.log("parent has 0 balls, going up");
                b = b.Parent;
            }

            if (b == null) return null;

            double min_dist = double.MaxValue;
            Slice nearest = null;

            foreach(Slice s in b.Slices)
            {
                double dist = Point2F.Distance(s.Center, center);
                if (dist < min_dist)
                {
                    min_dist = dist;
                    nearest = s;
                }
            }
            return nearest;
        }

        List<Circle2F> find_intersecting_balls(List<Circle2F> ballist, Circle2F ball)
        {
            List<Circle2F> result = new List<Circle2F>();
            foreach (Circle2F b in ballist)
            {
                double dist = Point2F.Distance(b.Center, ball.Center);
                if (b.Radius + ball.Radius <= dist + GENERAL_TOLERANCE)
                    continue;
                result.Add(b);
            }
            return result;
        }

        List<Circle2F> find_intersecting_balls(List<Circle2F> ballist, Arc2F arc)
        {
            List<Circle2F> result = new List<Circle2F>();
            foreach (Circle2F b in ballist)
            {
                double dist = Point2F.Distance(b.Center, arc.Center);
                if (b.Radius + arc.Radius <= dist + GENERAL_TOLERANCE)
                    continue;
                Line2F splitline = arc.CircleIntersect(b);
                if (splitline.p1.IsUndefined && splitline.p2.IsUndefined)
                    continue;
                result.Add(b);
            }
            return result;
        }

        List<Arc2F> filter_inner_arcs(List<Circle2F> ballist, List<Arc2F> segments)
        {
            List<Arc2F> result = new List<Arc2F>();

            foreach (Arc2F seg in segments)
            {
                bool is_inner = false;
                foreach (Circle2F ball in ballist)
                {
                    // XXX: crude, but will work in most cases
                    if (Point2F.Distance(ball.Center, seg.Midpoint) < ball.Radius)
                    {
                        is_inner = true;
                        break;
                    }
                }
                if (! is_inner)
                    result.Add(seg);
            }
            return result;
        }


        List<Slice> roll(Branch branch, List<Slice> ready_slices, RotationDirection dir, double min_segment_length)
        {
            List <Point2F> samples = sample_curve(branch.Curve, _sample_distance);

            Slice prev_slice = null;
            Point2F pending_slice_center = Point2F.Undefined;

            int i = 0;

            // initial slice
            if (branch.Parent != null)
            {
                prev_slice = find_nearest_slice(branch.Parent, samples[0]);
            }
            else
            {
                // top branch should always had a big circle at pt[0] !
                //XXX: verify it !
                Point2F pt = samples[0];
                double radius = get_mic_radius(pt);

                Slice s = new Slice(pt, radius, dir);
                branch.Slices.Add(s);
                ready_slices.Add(s);
                prev_slice = s;
                i += 1;
            }

            // XXX: lerp instead of skipping should be nice
            for (; i < samples.Count; i++)
            {
                Point2F pt = samples[i];
                double radius = get_mic_radius(pt);

                if (radius < _cutter_r)
                    continue;

                double max_slice_engage = Slice.Calc_max_engagement(pt, radius, prev_slice);

                if (max_slice_engage < _max_engagement)
                {
                    if (max_slice_engage < 0)
                        Host.log("Negative engagement detected !");

                    pending_slice_center = pt;
                    continue;
                }

                Slice s = new Slice(pt, radius, prev_slice, dir);

                if (s.Segments.Count == 0)
                {
                    Host.log("Undefined intersection (can't pass thru slot ?). Stopping slicing the branch.");
                    return branch.Slices;
                }
                else
                {
                    pending_slice_center = Point2F.Undefined;
                    branch.Slices.Add(s);
                    ready_slices.Add(s);
                    prev_slice = s;
                }
            }

            if (branch.End_is_leaf && (! pending_slice_center.IsUndefined))
            {
                double radius = get_mic_radius(pending_slice_center);
                Slice s = new Slice(pending_slice_center, radius, prev_slice, dir);
                branch.Slices.Add(s);
                ready_slices.Add(s);
            }

            return branch.Slices;
        }

        //XXX: slices are for debug
        List<Arc2F> segment_arc_by_balls(Arc2F basic, List<Circle2F> ballist, List<Entity> slices)
        {
            // split arc more to reduce air time

            List<Point2F> insect_points = new List<Point2F>();
            foreach (Circle2F b in ballist)
            {
                Line2F insect_line = basic.CircleIntersect(b);
                if (!insect_line.p1.IsUndefined)
                    insect_points.Add(insect_line.p1);
                if (!insect_line.p2.IsUndefined)
                    insect_points.Add(insect_line.p2);
            }


            Point2F[] insect_array = insect_points.ToArray();
            Array.Sort(insect_array, new Sweep_comparer(basic.P1, basic.Center, basic.Direction));


            /*
            foreach (Point2F pt in insect_array)
            {
                slices.Add(new Circle(pt, 1));
            }
            */

            List<Arc2F> segments = new List<Arc2F>();

            // XXX: wrong, wrong, wrong
            Arc2F remain = basic;
            for (int idx = 0; idx < insect_array.Length; idx++)
            {
                Arc2F[] split = remain.SplitAtPoint(insect_array[idx]);
//                if (Point2F.Distance(split[0].P1, split[0].P2) > GENERAL_TOLERANCE)
//                {
                    segments.Add(split[0]);
                    remain = split[1];
//                }
            }

//            if (Point2F.Distance(remain.P1, remain.P2) > GENERAL_TOLERANCE)
                segments.Add(remain);

            // XXX: Good segments may be filtered out !
             //segments = filter_inner_arcs(ballist, segments);

            //slices.Add(new Arc(basic_arc));

            return segments;
        }

        Polyline[] separate_mat_polys(Polyline[] medial)
        {
            ShapeList sl = new ShapeList(medial, false);
            Dictionary<Point2F, ShapeIntersection> insects = sl.GetIntersections(GENERAL_TOLERANCE);

            if (insects == null) return medial;

            Point2F[] insect_points = new List<Point2F>(insects.Keys).ToArray();
            List<Polyline> segments = new List<Polyline>();

            foreach (Polyline p in medial)
                segments.AddRange(p.SplitAtPoints(insect_points, GENERAL_TOLERANCE));

            return segments.ToArray();
        }

        public List<Entity> run()
        {
            List<Polyline> mat_segments = get_mat_segments();

            Host.log("segmented");

            Polyline[] mat = Polyline.Join(mat_segments.ToArray(), GENERAL_TOLERANCE);

            Host.log("joined");

            // do it twice, sometimes cambam miss intersections
            mat = separate_mat_polys(mat);
            mat = separate_mat_polys(mat);

            Branch root = get_branches(mat, _cutter_r * 2);
            List<Branch> traverse = root.Df_traverse();

//          foreach (Branch b in traverse)
//          {
//              CamBamUI.MainUI.ActiveView.CADFile.ActiveLayer.Entities.Add(b.Curve);
//          }
//
//          return new List<Entity>();

            List<Slice> ready_slices = new List<Slice>();

            foreach (Branch b in traverse)
            {
                roll(b, ready_slices, RotationDirection.CCW, _max_engagement / 2);
            }

            List<Entity> path = new List<Entity>();
            foreach (Branch b in traverse)
            {
                // XXX: for debug
                CamBamUI.MainUI.ActiveView.CADFile.ActiveLayer.Entities.Add(b.Curve.Clone());

                foreach (Slice s in b.Slices)
                {
                    foreach (Arc2F seg in s.Segments)
                    {
                        path.Add(new Arc(seg));
                    }
                }
            }

            return path;
        }

        public Pocket_generator(Region reg)
        {
            this._reg = reg;
        }
    }
}

//        List<Entity> get_path(Branch branch, List<Circle2F> ballist, bool is_cw, double min_segment_length)
//        {
//            List<Entity> slices = new List<Entity>();
//
//            if (branch.Balls.Count == 0) return slices;
//
//            Circle2F prev_ball;
//
//            int i = 0;
//
//            if (branch.Parent == null)
//            {
//                // XXX: here should be the spiral
//                Circle c = new Circle(branch.Balls[0]);
//                slices.Add(c);
//                prev_ball = branch.Balls[0];
//                ballist.Add(branch.Balls[0]);
//                i = 1;
//            }
//            else
//            {
//                prev_ball = find_nearest_ball(branch.Parent, point(branch.Curve.Points[0].Point));
//            }
//
//            for (; i < branch.Balls.Count; i++)
//            {
//                Circle2F ball = branch.Balls[i];
//                Line2F insects = ball.CircleIntersect(prev_ball);
//
//                if (insects.p1.IsUndefined || insects.p2.IsUndefined)
//                {
//                    // Probably it means there is unreachable slot
//                    // XXX: for  debug
//                    // XXX: throw exception here
//                    slices.Add(new Circle(ball.Center, 100));
//                    slices.Add(new Circle(prev_ball.Center, 120));
//                    Host.log("undefined intersection! i = {0}, r={1}, prev r={2}", i, ball.Radius, prev_ball.Radius);
//                }
//                else
//                {
//                    RotationDirection dir = is_cw ? RotationDirection.CW : RotationDirection.CCW;
//                    Arc2F basic_arc = new Arc2F(ball.Center, insects.p1, insects.p2, dir);
//
//                    //XXX: use dot product here !
//                    if (! basic_arc.VectorInsideArc(new Vector2F(prev_ball.Center, ball.Center)))
//                    {
//                        basic_arc = new Arc2F(ball.Center, insects.p2, insects.p1, dir);        // reverse it
//                    }
//
//                    List<Circle2F> more_insects = find_intersecting_balls(ballist, basic_arc);
//                    // XXX: would this rise exception if prev ball is not there ?
//                    more_insects.Remove(prev_ball);
//
//                    if (more_insects.Count == 0)
//                    {
//                        slices.Add(new Arc(basic_arc));
//                    }
//                    else
//                    {
//                        // just choose the ball with the longest trim chord for now
//
//                        /*
//                        Circle2F trimming_ball = more_insects[0];
//                        double max_chord = 0;
//                        for (int idx=1; idx < more_insects.Count; idx++)
//                        {
//                            Circle2F b = more_insects[idx];
//                            Line2F chord = basic_arc.CircleIntersect(b);
//                            // XXX: wrong !
//                            if (chord.p1.IsUndefined || chord.p2.IsUndefined)
//                                continue;
//
//                            double chord_len = chord.Length();
//                            if (chord_len > max_chord)
//                            {
//                                max_chord = chord_len;
//                                trimming_ball = b;
//                            }
//                        }
//
//                        more_insects.Clear();
//                        more_insects.Add(trimming_ball);
//                        */
//
//
//                        /*
//                        List<Arc2F> segments = segment_arc_by_balls(basic_arc, more_insects, slices);
//
//                        foreach (Arc2F seg in segments)
//                        {
//                            if (seg.GetPerimeter() > basic_arc.GetPerimeter())
//                            {
//                                Host.log("perimeter is bigger !");
//                            }
//
//                            if (seg.GetPerimeter() > min_segment_length)
//                            {
////                                slices.Add(new Arc(seg));
//                            }
//                        }
//                        */
//
//                        slices.Add(new Arc(basic_arc));
//                    }
//                }
//
//                prev_ball = ball;
//                ballist.Add(ball);
//            }
//            return slices;
//        }

