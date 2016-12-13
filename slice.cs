using System;
using System.Collections.Generic;

using CamBam.Geom;
using Geom;

// FYI: TED stands for Tool Engagement Depth (or Distance).
// TODO: define it

namespace Matmill
{
    enum Slice_placement
    {
        NORMAL,
        TOO_FAR,
        INSIDE_ANOTHER,
    }

    class Slice
    {
        private readonly Circle2F _ball;
        private List<Arc2F> _segments = new List<Arc2F>();        
        private readonly Slice_placement _placement;
        private double _entry_ted = 0;
        private double _mid_ted = 0;
        private double _max_ted = 0;

        public Circle2F Ball                { get { return _ball; } }
        public List<Arc2F> Segments         { get { return _segments; } }
        public Point2F Center               { get { return _ball.Center; } }
        public Point2F End                  { get { return _segments[_segments.Count - 1].P2; } }
        public Point2F Start                { get { return _segments[0].P1; } }
        public RotationDirection Dir        { get { return _segments[0].Direction; } }        
        public Slice_placement Placement    { get { return _placement; } }
        public double Max_ted               { get { return _max_ted; } }
        public double Radius                { get { return _ball.Radius; } }

        static private double angle_between_vectors(Vector2d v0, Vector2d v1, RotationDirection dir)
        {
            double angle = v0.Ccw_angle_to(v1);
            return (dir == RotationDirection.CCW) ? angle : (2.0 * Math.PI - angle);
        }

        private double calc_ted(Slice prev, Point2F tool_center, double tool_r)
        {
            Circle2F parent_wall = new Circle2F(prev.Center, prev.Radius + tool_r);

            // find the points tool touching parent wall
            Circle2F cut_circle = new Circle2F(tool_center, tool_r);
            Line2F insects = cut_circle.CircleIntersect(parent_wall);

            if (insects.p1.IsUndefined || insects.p2.IsUndefined)
            {
                Logger.err("no wall intersections #0");
                return 0;
            }

            // we're interested in tool head point, so choose more advanced point in mill direction

            Vector2d v1 = new Vector2d(prev.Center, insects.p1);
            Vector2d v_parent_to_tool_center = new Vector2d(prev.Center, tool_center);
            Point2F cut_head = v_parent_to_tool_center.Det(v1) * (int)this.Dir > 0 ? insects.p1 : insects.p2;

            // project headpoint to the center find TED
            Vector2d v_me_to_tool_center = new Vector2d(this.Center, tool_center);
            Vector2d v_me_to_cut_head = new Vector2d(this.Center, cut_head);

            double ted = this.Radius + tool_r - v_me_to_cut_head * v_me_to_tool_center.Unit();
            return ted > 0 ? ted : 0;
        }

        private void calc_teds(Slice prev, double tool_r)
        {
            // expand both slices to form walls
            Circle2F parent_wall = new Circle2F(prev.Center, prev.Radius + tool_r);
            Circle2F this_wall = new Circle2F(this.Center, this.Radius + tool_r);

            // find the point of cut start (intersection of walls)
            Line2F wall_insects = parent_wall.CircleIntersect(this_wall);

            if (wall_insects.p1.IsUndefined || wall_insects.p2.IsUndefined)
            {
                Logger.err("no wall intersections #1");
                return;
            }

            Vector2d v_move = new Vector2d(prev.Center, this.Center);
            Vector2d v1 = new Vector2d(prev.Center, wall_insects.p1);
            Point2F cut_tail = v_move.Det(v1) * (int)this.Dir < 0 ? wall_insects.p1 : wall_insects.p2;
            Vector2d v_tail = new Vector2d(this.Center, cut_tail);

            Point2F entry_tool_center = this.Center + (v_tail.Unit() * this.Radius).Point;
            _entry_ted = calc_ted(prev, entry_tool_center, tool_r);

            Point2F mid_tool_center = this.Center + (v_move.Unit() * this.Radius).Point;
            _mid_ted  = calc_ted(prev, mid_tool_center, tool_r);
        }

        public void Get_extrema(ref Point2F min, ref Point2F max)
        {            
            _segments[0].GetExtrema(ref min, ref max);

            // union of arcs extremas if there are several arcs
            for (int i = 1; i < _segments.Count; i++)
            {
                Point2F arcmin = Point2F.Undefined;
                Point2F arcmax = Point2F.Undefined;
                _segments[i].GetExtrema(ref arcmin, ref arcmax);

                min.X = Math.Min(min.X, arcmin.X);
                min.Y = Math.Min(min.Y, arcmin.Y);
                max.X = Math.Max(max.X, arcmax.X);
                max.Y = Math.Max(max.Y, arcmax.Y);
            }
        }

        public void Get_ball_extrema(ref Point2F min, ref Point2F max)
        {
            min = new Point2F(this.Center.X - this.Radius, this.Center.Y - this.Radius);
            max = new Point2F(this.Center.X + this.Radius, this.Center.Y + this.Radius);
        }

        public void Refine(Slice prev, List<Slice> colliding_slices, double end_clearance, double tool_r)
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
            // to guarantee the tool will never hit the unmilled area while rapiding between segments,
            // arc will always have original ends, trimming will happen in the middle only.
            // to prevent the tool from milling extra small end segments and minimize numeric errors at small tangents,
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
                if (s == prev)
                    continue;  // no reason to process it
                Line2F secant = arc.CircleIntersect(s.Ball);

                if (secant.p1.IsUndefined && secant.p2.IsUndefined)
                    continue;

                if (secant.p1.IsUndefined || secant.p2.IsUndefined)
                {
                    // single intersection
                    Point2F splitpt = secant.p1.IsUndefined ? secant.p2 : secant.p1;
                    if (arc.P1.DistanceTo(s.Center) < arc.P2.DistanceTo(s.Center))
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
                    if (check_arc.Midpoint.DistanceTo(s.Center) < s.Radius)
                    {
                        max_sweep = sweep;
                        max_secant = secant;
                    }
                }
            }

            if (max_sweep == 0)
                return;

            Arc2F head = new Arc2F(arc.Center, arc.P1, max_secant.p1, arc.Direction);
            Arc2F tail = new Arc2F(arc.Center, max_secant.p2, arc.P2, arc.Direction);

            _segments.Clear();
            _segments.Add(head);
            _segments.Add(tail);

            // recalculate max TED accounting for the new endpoints.
            // this TED is 'virtual' and averaged with previous max_ted to make arcs look more pretty

            // if ends of removed segment are at the same side of direction vector,
            // midpoint is still present, _max_ted is valid and is maximum for sure
            Vector2d v_move = new Vector2d(prev.Center, this.Center);
            Vector2d v_removed_p1 = new Vector2d(prev.Center, max_secant.p1);
            Vector2d v_removed_p2 = new Vector2d(prev.Center, max_secant.p2);

            if (v_move.Det(v_removed_p1) * v_move.Det(v_removed_p2) > 0) return;

            double ted_head_end = calc_ted(prev, max_secant.p1, tool_r);
            double ted_tail_start = calc_ted(prev, max_secant.p2, tool_r);

            double ted = Math.Max(ted_head_end, ted_tail_start);

            if (ted <= 0)
            {
                Logger.err("max TED vanished after refining the slice !");
                return;
            }

            ted = (ted + _mid_ted) / 2;
            _max_ted = Math.Max(ted, _entry_ted);
        }

        public void Append_leadin_and_leadout(double leadin_angle, double leadout_angle)
        {
            RotationDirection dir = Dir;
            Point2F start = Start;
            Point2F end = End;
            Point2F center = Center;

            Vector2d v1 = new Vector2d(center, start).Rotated(-(int)dir * leadin_angle);
            Vector2d v2 = new Vector2d(center, end).Rotated((int)dir * leadout_angle);

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

        private void create_arc_circle(Point2F p1, RotationDirection dir)
        {
            double seg_sweep = dir == RotationDirection.CW ? -2 * Math.PI / 3 : 2 * Math.PI / 3;

            Vector2d v1 = new Vector2d(this.Center, p1);
            Vector2d v2 = v1.Rotated(seg_sweep);
            Vector2d v3 = v1.Rotated(2 * seg_sweep);

            Point2F p2 = this.Center + (Point2F)v2;
            Point2F p3 = this.Center + (Point2F)v3;

            _segments.Clear();

            _segments.Add(new Arc2F(this.Center, p1, p2, dir));
            _segments.Add(new Arc2F(this.Center, p2, p3, dir));
            _segments.Add(new Arc2F(this.Center, p3, p1, dir));
        }

        public void Change_startpoint(Point2F p1)
        {
            if (Math.Abs((p1.DistanceTo(this.Center) - this.Radius) / this.Radius) > 0.001)
                throw new Exception("new startpoint is outside the initial circle");

            create_arc_circle(p1, Dir);
        }

        public Slice(Slice prev, Point2F center, double radius, RotationDirection dir, double tool_r, Point2F magnet)
        {            
            _ball = new Circle2F(center, radius);

            double dist = Point2F.Distance(center, prev.Center);
            double delta_r = this.Radius - prev.Radius;
            double coarse_ted = dist + delta_r;

            // 1) one ball is inside other, no intersections
            // 2) balls are spaced too far and do not intersect, TED is 0, distance is positive
            // 3) balls are intersect, TED is valid
            if (dist <= Math.Abs(delta_r))
            {
                _placement = Slice_placement.INSIDE_ANOTHER;
                return;
            }

            if (dist >= this.Radius + prev.Radius)
            {
                _placement = Slice_placement.TOO_FAR;
                return;
            }
            // TED can't be more >= tool diameter, this check prevents fails during the calculation of real angle-based TED
            if (coarse_ted > tool_r * 1.999)
            {
                _placement = Slice_placement.TOO_FAR;
                return;
            }

            Line2F insects = prev.Ball.CircleIntersect(this._ball);
            if (insects.p1.IsUndefined || insects.p2.IsUndefined)
            {
                // try to return meaningful result even if CB routine had failed (unlikely)
                Logger.err("no intersections found there intersections should be (_parent.Ball.CircleIntersect(_ball))");
                _placement = (dist <= this.Radius || dist <= prev.Radius) ? Slice_placement.INSIDE_ANOTHER : Slice_placement.TOO_FAR;
                return;
            }

            _placement = Slice_placement.NORMAL;

            Vector2d v_move = new Vector2d(prev.Center, this.Center);
            Vector2d v1 = new Vector2d(prev.Center, insects.p1);
            RotationDirection default_dir = v_move.Det(v1) > 0 ? RotationDirection.CW : RotationDirection.CCW;

            if (dir == RotationDirection.Unknown)
            {
                if (magnet.IsUndefined)
                {
                    dir = RotationDirection.CCW;    // just something
                }
                else
                {
                    if (insects.p1.DistanceTo(magnet) < insects.p2.DistanceTo(magnet))  // match, use existing dir
                        dir = default_dir;
                    else
                        dir = (default_dir == RotationDirection.CCW) ? RotationDirection.CW : RotationDirection.CCW;   // flip
                }
            }

            Arc2F arc;
            if (default_dir == dir)
                arc = new Arc2F(this.Center, insects.p1, insects.p2, dir);
            else
                arc = new Arc2F(this.Center, insects.p2, insects.p1, dir);

            _segments.Add(arc);

            calc_teds(prev, tool_r);

            if (_mid_ted <= 0)
            {
                Logger.warn("forced to patch mid_ted");
                _mid_ted = coarse_ted;    // shouldn't be, but ok
            }

            _max_ted = Math.Max(_mid_ted, _entry_ted);
        }

        public Slice(Point2F center, double radius, RotationDirection dir)
        {
            _ball = new Circle2F(center, radius);
            _placement = Slice_placement.NORMAL;
            create_arc_circle(new Point2F(center.X + radius, center.Y), dir);
        }
    }

}