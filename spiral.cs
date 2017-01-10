using System;
using System.Collections.Generic;

using CamBam.Geom;
using Geom;

namespace Matmill
{
    class Spiral_generator
    {
        private static double get_ted(double prev_r, double this_r, double tool_r)
        {
            double prev_wall_r = prev_r + tool_r;
            double this_wall_r = this_r + tool_r;

            if (this_r - prev_wall_r >= tool_r)
                return double.MaxValue;

            if (prev_wall_r - this_r >= tool_r)
                return double.MinValue;

            // get intersection of two cicles = cutter circle and prev wall
            Circle2F prev_wall = new Circle2F(new Point2F(0, 0), prev_wall_r);
            Circle2F tool_circle = new Circle2F(new Point2F(this_r, 0), tool_r);

            Line2F insects = prev_wall.CircleIntersect(tool_circle);
            if (insects.p1.IsUndefined || insects.p2.IsUndefined)
                return 0;

            double ted = this_wall_r - insects.p1.X;

            return ted;
        }

        public static double Calc_reverse_spacing(double this_r, double tool_r, double needed_ted, double tolerance)
        {
            double left = Math.Max(this_r - tool_r * 1.99, 0);
            double right = this_r * 0.99;
            double mid;

            while (true)
            {
                mid = (left + right) / 2;
                double ted = get_ted(mid, this_r, tool_r);

                if (ted == double.MaxValue || ted > needed_ted) // too far
                    left = mid;
                else if (ted == double.MinValue || ted < needed_ted) // too close
                    right = mid;
                else
                    break;  // ted = needed ted. unlikely, but ok

                if (Math.Abs(left - right) < tolerance)   // range has shrinked, stop search
                    break;
            }

            return this_r - mid;
        }

        // simple Archimedean spiral:
        // r = a * theta + c
        //
        // in cartesian coordinates:
        // x = center_x + (a * theta + c) * cos(theta)
        // y = center_y + (a * theta + c) * sin(theta)
        //
        // tangent vector (derivate relative to theta):
        // x' = a * cos(theta) - sin(theta) * r
        // y' = a * sin(theta) + cos(theta) * r

        // we output biarcs to approximate spiral, 6 biarcs (12 arcs) per loop.
        // real number of biarcs would be more, since in general case there would be incomplete loops
        // last biarc is exact to the endpoint

        // if start tangent is defined, spiral will start from it (spacing will be reduced a little to make a fit).
        // otherwise start tangent is choosed automatically and spacing is exact.
        public static List<Biarc2d> Gen_archimedean_spiral(Point2F center, Point2F end, Vector2d start_tangent, double spacing, RotationDirection dir)
        {
            double r_max = center.DistanceTo(end);
            Vector2d v_end = new Vector2d(center, end);


            double theta_end = v_end.Ccw_angle;
            double theta_step = 2 * Math.PI / 6;
            double a = spacing / (2 * Math.PI);
            double theta_start;

            if (dir == RotationDirection.CW)
            {
                a = -a;
                theta_step = -theta_step;
            }

            if (double.IsNaN(start_tangent.X))
            {
                theta_start = theta_end - r_max / a;
            }
            else
            {
                theta_start = start_tangent.Ccw_angle;
                double candidate_a;

                if (dir == RotationDirection.CCW)
                {
                    while (theta_start >= theta_end)
                        theta_start -= 2 * Math.PI;

                    while (true)
                    {
                        candidate_a = r_max / (theta_end - theta_start);
                        if (candidate_a <= a)
                            break;
                        theta_start -= 2 * Math.PI;
                    }
                }
                else
                {
                    while (theta_start <= theta_end)
                        theta_start += 2 * Math.PI;

                    while (true)
                    {
                        candidate_a = r_max / (theta_end - theta_start);
                        if (candidate_a >= a)
                            break;
                        theta_start += 2 * Math.PI;
                    }
                }
                a = candidate_a;
            }

            int nsegments = (int)Math.Floor((theta_end - theta_start) / theta_step);

            double c = -theta_start * a;

            Point2F p1 = new Point2F();
            Vector2d t1 = new Vector2d();
            List<Biarc2d> spiral = new List<Biarc2d>();

            for (int segidx = 0; segidx < nsegments; segidx++)
            {
                double theta = theta_start + segidx * theta_step;
                double r = a * theta + c;

                double sin = Math.Sin(theta);
                double cos = Math.Cos(theta);

                Point2F p2 = center + new Point2F(r * cos, r * sin);
                Vector2d t2 = new Vector2d(a * cos - r * sin, a * sin + r * cos).Unit();

                if (dir == RotationDirection.CW)
                    t2 = t2.Inverted();

                if (segidx != 0)
                    spiral.Add(new Biarc2d(p1, t1, p2, t2));

                p1 = p2;
                t1 = t2;
            }

            Vector2d t_end = v_end.Normal().Unit();
            if (dir == RotationDirection.CW)
                t_end = t_end.Inverted();

            spiral.Add(new Biarc2d(p1, t1, end, t_end));

            return spiral;
        }
    }
}