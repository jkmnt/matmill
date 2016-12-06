using System;
using System.Collections.Generic;

using CamBam.Geom;
using Geom;

namespace Matmill
{
    class Spiral_generator
    {
        // simple Archimedean spiral:
        // r = a * theta
        //
        // in cartesian coordinates:
        // x = center_x + a * theta * cos(theta + theta_offset)
        // y = center_y + a * theta * sin(theta + theta_offset)
        //
        // tangent vector (derivate relative to theta):
        // x' = a * ( cos(theta + theta_offset) - theta * sin(theta + theta_offset))
        // y' = a * ( sin(theta + theta_offset) + theta * cos(theta + theta_offset))

        // we output biarcs to approximate spiral, 6 biarcs (12 arcs) per loop.
        // real number of biarcs would be more, since in general case there would be incomplete loops
        // last biarc is exact to the endpoint
        public static List<Biarc2d> Gen_archimedean_spiral(Point2F center, Point2F end, double spacing, RotationDirection dir)
        {
            Vector2d v_end = new Vector2d(center, end);
            double theta_end = Math.Atan2(v_end.Y, v_end.X);

            double r_max = center.DistanceTo(end);
            double a = spacing / (2 * Math.PI);
            if (dir == RotationDirection.CW)
                a = -a;

            double theta_stop = r_max / a;
            int nloops = (int)Math.Ceiling(Math.Abs(theta_stop) / (2 * Math.PI));
            int nsectors = nloops * 6;
            double theta_per_sector = theta_stop / nsectors;
            double theta_offset = - theta_stop + theta_end;

            Point2F p1 = new Point2F();
            Vector2d t1 = new Vector2d();

            List<Biarc2d> spiral = new List<Biarc2d>();

            for (int sector = 0; sector < nsectors; sector++)
            {
                double theta = sector * theta_per_sector;                
                double r = a * theta;
                double sin = Math.Sin(theta + theta_offset);
                double cos = Math.Cos(theta + theta_offset);

                Point2F p2 = new Point2F(center.X + r * cos, center.Y + r * sin);
                Vector2d t2 = new Vector2d(a * cos - r * sin, a * sin + r * cos).Unit();

                if (dir == RotationDirection.CW)
                    t2 = t2.Inverse();

                if (sector > 0)
                    spiral.Add(new Biarc2d(p1, t1, p2, t2));

                p1 = p2;
                t1 = t2;
            }

            Vector2d t_end = v_end.Normal().Unit();
            if (dir == RotationDirection.CW)
                t_end = t_end.Inverse();

            spiral.Add(new Biarc2d(p1, t1, end, t_end));

            return spiral;
        }
    }
}