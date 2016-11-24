using System;
using System.Collections;
using System.Collections.Generic;

using CamBam.Geom;

namespace Matmill
{
    class Slice
    {
        private Circle2F _ball;
        private List<Arc2F> _segments = new List<Arc2F>();
        private Slice _prev_slice;
        private double _max_engagement;
        private double _dist;

        public Circle2F Ball            { get { return _ball; } }
        public double Dist              { get { return _dist; } }

        public Point2F Center           { get { return _ball.Center; } }
        public double Radius            { get { return _ball.Radius; } }
        public Slice Prev               { get { return _prev_slice; } }
        public double Max_engagement    { get { return _max_engagement; } }
        public List<Arc2F> Segments     { get { return _segments; } }
        public RotationDirection Dir    { get { return _segments.Count > 0 ? _segments[0].Direction : RotationDirection.Unknown; } }

        static private double angle_between_vectors(Vector2F v0, Vector2F v1, RotationDirection dir)
        {
            double angle = Math.Atan2(Vector2F.Determinant(v0, v1), Vector2F.DotProduct(v0, v1));
            if (angle < 0)
                angle += 2.0 * Math.PI;
            return (dir == RotationDirection.CCW) ? angle : (2.0 * Math.PI - angle);
        }

        static private double calc_radial_engagement(Circle2F prev_wall, Circle2F this_wall, Point2F cutter_center, double cutter_r, RotationDirection dir)
        {
            // find the points cutter touch previous wall
            Circle2F cut_circle = new Circle2F(cutter_center, cutter_r);
            Line2F insects = cut_circle.CircleIntersect(prev_wall);

            if (insects.p1.IsUndefined || insects.p2.IsUndefined)
            {
                Host.err("no wall intersections #0");
                return 0;
            }

            // we're interested in cutter head point, so choose more advanced point in mill direction
            Point2F cut_head;
            Vector2F v1 = new Vector2F(prev_wall.Center, insects.p1);
            Vector2F v2 = new Vector2F(prev_wall.Center, insects.p2);
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

        private double calc_true_engagement(double cutter_r, RotationDirection dir)
        {
            // expand both slices
            Circle2F prev_wall = new Circle2F(_prev_slice.Center, _prev_slice.Radius + cutter_r);
            Circle2F this_wall = new Circle2F(_ball.Center, _ball.Radius + cutter_r);

            Line2F insects = prev_wall.CircleIntersect(this_wall);

            if (insects.p1.IsUndefined || insects.p2.IsUndefined)
                return 0;

            Point2F cut_tail;

            Arc2F test_arc = new Arc2F(_ball.Center, insects.p1, insects.p2, dir);
            if (test_arc.VectorInsideArc(new Vector2F(_prev_slice.Center, _ball.Center)))            
                cut_tail = insects.p1;            
            else            
                cut_tail = insects.p2;            

            Line2F tail_ray = new Line2F(_ball.Center, cut_tail);
            insects = _ball.LineIntersect(tail_ray);

            // shouldn't be
            if (insects.p1.IsUndefined && insects.p2.IsUndefined)
                return 0;

            // center of initial cut circle
            Point2F cutter_center = insects.p1.IsUndefined ? insects.p2 : insects.p1;
            double initial_engagement = calc_radial_engagement(prev_wall, this_wall, cutter_center, cutter_r, dir);

            Arc2F arc = _segments[0];
            double central_engagement = calc_radial_engagement(prev_wall, this_wall, arc.Midpoint, cutter_r, dir);

            return Math.Max(initial_engagement, central_engagement);
        }

        public void Get_extrema(ref Point2F min, ref Point2F max)
        {
            // special processing for the very first slice, treat it as ball
            if (_prev_slice == null)
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

        public void Refine(List<Slice> colliding_slices, double end_clearance, double seg_engagement_derating)
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
                if (s == _prev_slice)
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
                Vector2F v_p1 = new Vector2F(arc.Center, arc.P1);
                Vector2F v_ins1 = new Vector2F(arc.Center, secant.p1);
                Vector2F v_ins2 = new Vector2F(arc.Center, secant.p2);

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
            // TOD: should we apply opposite derating if base engagement is valid ?
            if (removed.VectorInsideArc(new Vector2F(arc.Center, arc.Midpoint)))
            {
                double e0 = _prev_slice.Center.DistanceTo(max_secant.p1) - _prev_slice.Radius;
                double e1 = _prev_slice.Center.DistanceTo(max_secant.p2) - _prev_slice.Radius;

                _max_engagement = (1 - seg_engagement_derating) * Math.Max(e0, e1) + _max_engagement * seg_engagement_derating;
            }
        }

        public void Flip_dir()
        {
            for (int i = 0; i < _segments.Count; i++)
            {
                Arc2F arc = _segments[i];
                _segments[i] = new Arc2F(arc.Center, arc.P2, arc.P1, arc.Direction == RotationDirection.CCW ? RotationDirection.CW : RotationDirection.CCW);
            }
            _segments.Reverse();
        }

        public Slice(Slice prev_slice, Slice last_slice, Point2F center, double radius, double cutter_r)
        {
            _prev_slice = prev_slice;
            _ball = new Circle2F(center, radius);
            _dist = Point2F.Distance(center, prev_slice.Center);
            RotationDirection dir = _prev_slice.Dir;

            double delta_r = radius - prev_slice.Radius;
            double sum_r = radius + prev_slice.Radius;

            // 1) one ball is inside other, no intersections - engagement is 0, mark the distance as negative
            // 2) balls are spaced too far and do not intersect, engagement is 0, distance is positive
            // 3) balls are intersect, engagement is valid

            if (_dist <= Math.Abs(delta_r))
            {
                _max_engagement = 0;
                _dist = -_dist;
                return;
            }

            if (_dist >= sum_r)
            {
                _max_engagement = 0;
                return;
            }

            _max_engagement = _dist + delta_r;

            // engagement can't be more >= cutter diameter, this check prevents fails during the calculation of real angle-based engagement
            if (_max_engagement > cutter_r * 1.999)
            {
                _max_engagement = 0;
                return;
            }

            Line2F insects = _prev_slice.Ball.CircleIntersect(_ball);

            if (insects.p1.IsUndefined || insects.p2.IsUndefined)
            {
                // try to return meaningful result even if CB routine had failed (unlikely)
                Host.err("no intersections found there intersections should be (_prev_slice.Ball.CircleIntersect(_ball))");
                if (_dist <= radius || _dist <= prev_slice.Radius)
                    _dist = -Math.Abs(_dist);
                _max_engagement = 0;
                return;
            }

            Arc2F arc = new Arc2F(_ball.Center, insects.p1, insects.p2, dir);

            if (!arc.VectorInsideArc(new Vector2F(_prev_slice.Center, _ball.Center)))
                arc = new Arc2F(_ball.Center, insects.p2, insects.p1, dir);             // flip arc start/end, preserving same direction

            _segments.Add(arc);

            double true_engagement = calc_true_engagement(cutter_r, dir);            
            _max_engagement = Math.Max(_max_engagement, true_engagement);
        }

        public Slice(Point2F center, double radius, RotationDirection dir)
        {
            _ball = new Circle2F(center, radius);
            _max_engagement = 0;
            _dist = 0;

            if (dir == RotationDirection.CCW)
            {
                _segments.Add(new Arc2F(center, radius, 0, 120));
                _segments.Add(new Arc2F(center, radius, 120, 120));
                _segments.Add(new Arc2F(center, radius, 240, 120));
            }
            else
            {
                _segments.Add(new Arc2F(center, radius, 0, -120));
                _segments.Add(new Arc2F(center, radius, 240, -120));
                _segments.Add(new Arc2F(center, radius, 120, -120));
            }
        }
    }
}