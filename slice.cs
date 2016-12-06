using System;
using System.Collections.Generic;

using CamBam.Geom;
using Geom;

namespace Matmill
{
    class Slice
    {
        private readonly Circle2F _ball;
        private List<Arc2F> _segments = new List<Arc2F>();
        private readonly Slice _parent;
        private readonly double _dist = 0;
        private double _entry_engagement = 0;
        private double _mid_engagement = 0;
        private double _max_engagement = 0;

        public Circle2F Ball            { get { return _ball; } }
        public double Dist              { get { return _dist; } }

        public Point2F Center           { get { return _ball.Center; } }
        public double Radius            { get { return _ball.Radius; } }
        public Slice Parent             { get { return _parent; } }
        public double Max_engagement    { get { return _max_engagement; } }
        public List<Arc2F> Segments     { get { return _segments; } }
        public Point2F Start            { get { return _segments[0].P1; } }
        public Point2F End              { get { return _segments[_segments.Count - 1].P2; } }
        public RotationDirection Dir    { get { return _segments[0].Direction; } }

        static private double angle_between_vectors(Vector2d v0, Vector2d v1, RotationDirection dir)
        {
            double angle = v0.Ccw_angle_to(v1);
            return (dir == RotationDirection.CCW) ? angle : (2.0 * Math.PI - angle);
        }

        static private double calc_radial_engagement(Circle2F parent_wall, Circle2F this_wall, Point2F cutter_center, double cutter_r, RotationDirection dir)
        {
            // find the points cutter touch parent wall
            Circle2F cut_circle = new Circle2F(cutter_center, cutter_r);
            Line2F insects = cut_circle.CircleIntersect(parent_wall);

            if (insects.p1.IsUndefined || insects.p2.IsUndefined)
            {
                Logger.err("no wall intersections #0");
                return 0;
            }

            // we're interested in cutter head point, so choose more advanced point in mill direction
            Point2F cut_head;
            Vector2d v1 = new Vector2d(parent_wall.Center, insects.p1);
            Vector2d v2 = new Vector2d(parent_wall.Center, insects.p2);
            if (angle_between_vectors(v1, v2, dir) < Math.PI)
                cut_head = insects.p2;
            else
                cut_head = insects.p1;

            // project headpoint to the tail ray and find radial engagement
            Vector2F cut_tail_ray = new Vector2F(this_wall.Center, cutter_center);
            Vector2F cut_head_ray = new Vector2F(this_wall.Center, cut_head);

            double engagement = this_wall.Center.DistanceTo(cutter_center) + cutter_r - Vector2F.DotProduct(cut_head_ray, cut_tail_ray.Unit());

            if (engagement < 0)
                engagement = 0;

            return engagement;
        }

        private void calc_engagements(double cutter_r)
        {
            RotationDirection dir = _segments[0].Direction;

            // expand both slices to form walls
            Circle2F parent_wall = new Circle2F(_parent.Center, _parent.Radius + cutter_r);
            Circle2F this_wall = new Circle2F(_ball.Center, _ball.Radius + cutter_r);

            Line2F insects = parent_wall.CircleIntersect(this_wall);

            if (insects.p1.IsUndefined || insects.p2.IsUndefined)
            {
                Logger.err("no wall intersections #1");
                return;
            }

            Arc2F test_arc = new Arc2F(_ball.Center, insects.p1, insects.p2, dir);
            Point2F cut_tail = test_arc.VectorInsideArc(new Vector2F(_parent.Center, _ball.Center)) ? insects.p1 : insects.p2;
            Line2F tail_ray = new Line2F(_ball.Center, cut_tail);
            insects = _ball.LineIntersect(tail_ray);

            if (insects.p1.IsUndefined && insects.p2.IsUndefined)
            {
                Logger.err("no wall intersections #2");
                return;
            }

            // center of initial cut circle
            Point2F cutter_center;

            cutter_center = insects.p1.IsUndefined ? insects.p2 : insects.p1;
            _entry_engagement = calc_radial_engagement(parent_wall, this_wall, cutter_center, cutter_r, dir);

            cutter_center = _segments[0].Midpoint;
            _mid_engagement  = calc_radial_engagement(parent_wall, this_wall, cutter_center, cutter_r, dir);
        }

        public void Get_extrema(ref Point2F min, ref Point2F max)
        {
            // special processing for the very first slice, treat it as ball
            if (_parent == null)
            {
                Get_ball_extrema(ref min, ref max);
                return;
            }

            Arc2F arc;
            if (_segments.Count == 1)
                arc = _segments[0];
            else
                arc = new Arc2F(_segments[0].P1, _segments[_segments.Count - 1].P2, _segments[0].Center, _segments[0].Direction);

            arc.GetExtrema(ref min, ref max);
        }

        public void Get_ball_extrema(ref Point2F min, ref Point2F max)
        {
            min = new Point2F(_ball.Center.X - _ball.Radius, _ball.Center.Y - _ball.Radius);
            max = new Point2F(_ball.Center.X + _ball.Radius, _ball.Center.Y + _ball.Radius);
        }

        public void Refine(List<Slice> colliding_slices, double end_clearance, double seg_engagement_derating, double cutter_r)
        {
            double clearance = end_clearance;

            // check if arc is small. refining is worthless in this case
            // criterion for smallness: there should be at least 4 segments with chord = clearance, plus
            // one segment to space ends far enough. A pentagon with a 5 segments with edge length = clearance
            // will define the min radius of circumscribed circle. clearance = 2 * R * sin (Pi / 5),
            // R = clearance / 2 / sin (Pi / 5)

            if (_segments.Count != 1)
                throw new Exception("attempt to refine slice with n segments != 1");

            Arc2F arc = _segments[0];

            double r_min = clearance / 2 / Math.Sin(Math.PI / 5.0);
            if (arc.Radius <= r_min)
                return;

            if (colliding_slices.Contains(this))
                throw new Exception("attempt to collide slice with itself");

            // now apply the colliding slices. to keep things simple and robust, we apply just one slice - the one who trims
            // us most (removed length of arc is greatest).

            // end clearance adjustment:
            // to guarantee the cutter will never hit the unmilled area while rapiding between segments,
            // arc will always have original ends, trimming will happen in the middle only.
            // to prevent the cutter from milling extra small end segments and minimize numeric errors at small tangents,
            // original ends would always stick at least for a clearance (chordal) length.
            // i.e. if the point of intersection of arc and colliding circle is closer than clearance to the end,
            // it is moved to clearance distance.

            // there is a two cases of intersecting circles: with single intersection and with a double intersection.
            // double intersections splits arc to three pieces (length of the middle part is the measure),
            // single intesections splits arc in two (the part inside the circle is removed, its length is the measure).
            // in both cases the intersection points are subject to "end clearance adjustment".
            // single intersections are transformed to the double intersections, second point being one of the end clearances.

            // TODO: calculate clearance point the right way, with math :-)
            Line2F c1_insects = arc.CircleIntersect(new Circle2F(arc.P1, clearance));
            Line2F c2_insects = arc.CircleIntersect(new Circle2F(arc.P2, clearance));
            Point2F c1 = c1_insects.p1.IsUndefined ? c1_insects.p2 : c1_insects.p1;
            Point2F c2 = c2_insects.p1.IsUndefined ? c2_insects.p2 : c2_insects.p1;

            Line2F max_secant = new Line2F();
            double max_sweep = 0;

            foreach (Slice s in colliding_slices)
            {
                if (s == _parent)
                    continue;  // no reason to process it
                Line2F secant = arc.CircleIntersect(s.Ball);

                if (secant.p1.IsUndefined && secant.p2.IsUndefined)
                    continue;

                if (secant.p1.IsUndefined || secant.p2.IsUndefined)
                {
                    // single intersection
                    Point2F splitpt = secant.p1.IsUndefined ? secant.p2 : secant.p1;
                    if (arc.P1.DistanceTo(s.Ball.Center) < arc.P2.DistanceTo(s.Ball.Center))
                    {
                        if (splitpt.DistanceTo(arc.P1) < clearance)
                            continue;  // nothing to remove
                        else if (splitpt.DistanceTo(arc.P2) < clearance)
                            secant = new Line2F(c1, c2);
                        else
                            secant = new Line2F(c1, splitpt);
                    }
                    else
                    {
                        // remove second segment
                        if (splitpt.DistanceTo(arc.P2) < clearance)
                            continue;
                        else if (splitpt.DistanceTo(arc.P1) < clearance)
                            secant = new Line2F(c1, c2);
                        else
                            secant = new Line2F(splitpt, c2);
                    }
                }
                else
                {
                    // double intersection
                    if (secant.p1.DistanceTo(arc.P1) < clearance)
                        secant.p1 = c1;
                    else if (secant.p1.DistanceTo(arc.P2) < clearance)
                        secant.p1 = c2;

                    if (secant.p2.DistanceTo(arc.P1) < clearance)
                        secant.p2 = c1;
                    else if (secant.p2.DistanceTo(arc.P2) < clearance)
                        secant.p2 = c2;
                }

                if (secant.p1.DistanceTo(secant.p2) < clearance * 2) // segment is too short, ignore it
                    continue;

                // sort insects by sweep (already sorted for single, may be unsorted for the double)
                Vector2d v_p1 = new Vector2d(arc.Center, arc.P1);
                Vector2d v_ins1 = new Vector2d(arc.Center, secant.p1);
                Vector2d v_ins2 = new Vector2d(arc.Center, secant.p2);

                double sweep = angle_between_vectors(v_ins1, v_ins2, arc.Direction);

                if (angle_between_vectors(v_p1, v_ins1, arc.Direction) > angle_between_vectors(v_p1, v_ins2, arc.Direction))
                {
                    secant = new Line2F(secant.p2, secant.p1);
                    sweep = 2.0 * Math.PI - sweep;
                }

                if (sweep > max_sweep)
                {
                    // ok, a last check - removed arc midpoint should be inside the colliding circle
                    Arc2F check_arc = new Arc2F(arc.Center, secant.p1, secant.p2, arc.Direction);
                    if (check_arc.Midpoint.DistanceTo(s.Ball.Center) < s.Ball.Radius)
                    {
                        max_sweep = sweep;
                        max_secant = secant;
                    }
                }
            }

            if (max_sweep == 0)
                return;

            Arc2F start = new Arc2F(arc.Center, arc.P1, max_secant.p1, arc.Direction);
            Arc2F removed = new Arc2F(arc.Center, max_secant.p1, max_secant.p2, arc.Direction);
            Arc2F end = new Arc2F(arc.Center, max_secant.p2, arc.P2, arc.Direction);

            _segments.Clear();
            _segments.Add(start);
            _segments.Add(end);


            // recalculate engagement if base engagement is no longer valid (midpoint vanished with the removed middle segment).
            // this engagement is 'virtual' and averaged with base to reduce stress on cutter abruptly entering the wall
            // TODO: is it really needed ?
            if (! removed.VectorInsideArc(new Vector2F(arc.Center, arc.Midpoint))) return;

            // expand both slices
            Circle2F parent_wall = new Circle2F(_parent.Center, _parent.Radius + cutter_r);
            Circle2F this_wall = new Circle2F(_ball.Center, _ball.Radius + cutter_r);

            double e0 = calc_radial_engagement(parent_wall, this_wall, max_secant.p1, cutter_r, arc.Direction);
            double e1 = calc_radial_engagement(parent_wall, this_wall, max_secant.p2, cutter_r, arc.Direction);

            double max_eng = Math.Max(e0, e1);

            if (max_eng <= 0)
            {
                Logger.err("max engagement vanished after refining the slice !");
                return;
            }

            max_eng = (1 - seg_engagement_derating) * max_eng + _mid_engagement * seg_engagement_derating;
            _max_engagement = Math.Max(max_eng, _entry_engagement);
        }

        public void Append_leadin_and_leadout(double leadin_angle, double leadout_angle)
        {
            RotationDirection dir = Dir;
            Point2F start = Start;
            Point2F end = End;
            Point2F center = Center;

            Vector2d v1 = new Vector2d(center, start).Rotated(dir == RotationDirection.CCW ? -leadin_angle : leadin_angle);
            Vector2d v2 = new Vector2d(center, end).Rotated(dir == RotationDirection.CCW ? leadout_angle : -leadout_angle);

            if (_segments.Count == 1)   // common case
            {
                _segments[0] = new Arc2F(center, center + v1.Point, center + v2.Point, dir);
            }
            else
            {
                _segments[0] = new Arc2F(center, center + v1.Point, _segments[0].P2, dir);
                _segments[_segments.Count - 1] = new Arc2F(center, _segments[_segments.Count - 1].P1, center + v2.Point, dir);
            }
        }

        private void create_initial_slice(Point2F center, Point2F p1, RotationDirection dir)
        {
            double seg_sweep = dir == RotationDirection.CW ? -2 * Math.PI / 3 : 2 * Math.PI / 3;

            Vector2d v1 = new Vector2d(center, p1);
            Vector2d v2 = v1.Rotated(seg_sweep);
            Vector2d v3 = v1.Rotated(2 * seg_sweep);

            Point2F p2 = center + (Point2F)v2;
            Point2F p3 = center + (Point2F)v3;

            _segments.Clear();

            _segments.Add(new Arc2F(center, p1, p2, dir));
            _segments.Add(new Arc2F(center, p2, p3, dir));
            _segments.Add(new Arc2F(center, p3, p1, dir));
        }

        public void Change_startpoint(Point2F p1)
        {
            if (_parent != null)
                throw new Exception("startpoint may be changed for the root slice only");
            if (Math.Abs((p1.DistanceTo(Center) - Radius) / Radius) > 0.001)
                throw new Exception("new startpoint is outside the initial circle");

            create_initial_slice(Center, p1, Dir);
        }

        public Slice(Slice parent, Point2F center, double radius, RotationDirection dir, double cutter_r, Slice last_slice)
        {
            _parent = parent;
            _ball = new Circle2F(center, radius);
            _dist = Point2F.Distance(center, parent.Center);

            double delta_r = radius - parent.Radius;
            double coarse_engagement = _dist + delta_r;

            // 1) one ball is inside other, no intersections - engagement is 0, mark the distance as negative
            // 2) balls are spaced too far and do not intersect, engagement is 0, distance is positive
            // 3) balls are intersect, engagement is valid
            if (_dist <= Math.Abs(delta_r))
            {
                _dist = -_dist;
                return;
            }
            if (_dist >= radius + parent.Radius) return;

            // engagement can't be more >= cutter diameter, this check prevents fails during the calculation of real angle-based engagement
            if (coarse_engagement > cutter_r * 1.999) return;

            Line2F insects = _parent.Ball.CircleIntersect(_ball);
            if (insects.p1.IsUndefined || insects.p2.IsUndefined)
            {
                // try to return meaningful result even if CB routine had failed (unlikely)
                Logger.err("no intersections found there intersections should be (_parent.Ball.CircleIntersect(_ball))");
                if (_dist <= radius || _dist <= parent.Radius)
                    _dist = -Math.Abs(_dist);
                return;
            }

            Arc2F arc;

            bool should_choose_direction = false;
            if (dir == RotationDirection.Unknown)
            {
                dir = RotationDirection.CW;
                if (last_slice != null)
                    should_choose_direction = true;
            }

            if (! should_choose_direction)
            {
                arc = new Arc2F(_ball.Center, insects.p1, insects.p2, dir);
                if (!arc.VectorInsideArc(new Vector2F(_parent.Center, _ball.Center)))
                    arc = new Arc2F(_ball.Center, insects.p2, insects.p1, dir);         // flip arc start/end, preserving same direction
            }
            else
            {
                if (last_slice.End.DistanceTo(insects.p1) < last_slice.End.DistanceTo(insects.p2))
                    arc = new Arc2F(_ball.Center, insects.p1, insects.p2, dir);
                else
                    arc = new Arc2F(_ball.Center, insects.p2, insects.p1, dir);

                if (!arc.VectorInsideArc(new Vector2F(_parent.Center, _ball.Center)))
                {
                    dir = RotationDirection.CCW;
                    arc = new Arc2F(_ball.Center, arc.P1, arc.P2, dir);                 // flip direction, preserving same start/end
                }
            }

            _segments.Add(arc);

            calc_engagements(cutter_r);

            if (_mid_engagement <= 0)
            {
                Logger.warn("forced to patch mid_engagement");
                _mid_engagement = coarse_engagement;    // shouldn't be, but ok
            }

            _max_engagement = Math.Max(_mid_engagement, _entry_engagement);
        }

        public Slice(Point2F center, double radius, RotationDirection dir)
        {
            _ball = new Circle2F(center, radius);
            create_initial_slice(center, new Point2F(center.X + radius, center.Y), dir);
        }
    }
}