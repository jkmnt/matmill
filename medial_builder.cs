using System;
using System.Collections.Generic;

using CamBam.Geom;

using TriangleNet;

namespace Matmill
{
    interface Medial_branch
    {
        Point2F Start { get; }
        Point2F End { get; }
        double Shallow_distance { get; }
        double Deep_distance { get; }

        void Add_point(Point2F pt);
        Medial_branch Spawn_child();
        void Attach_to_parent();
        void Postprocess();
    }

    class Medial_builder
    {        
        private const bool ANALIZE_INNER_INTERSECTIONS = false;

        private static List<Line2F> get_medial_axis_segments(Topographer topo, List<Point2F>samples, double general_tolerance)
        {
            TriangleNet.Meshing.GenericMesher mesher = new TriangleNet.Meshing.GenericMesher();
            List<TriangleNet.Geometry.Vertex> vertices = new List<TriangleNet.Geometry.Vertex>();
            foreach (Point2F pt in samples)
            {
                vertices.Add(new TriangleNet.Geometry.Vertex(pt.X, pt.Y));
            }

            TriangleNet.Mesh mesh = (TriangleNet.Mesh) mesher.Triangulate(vertices);

            TriangleNet.Voronoi.StandardVoronoi voronoi = new TriangleNet.Voronoi.StandardVoronoi(mesh);

            Logger.log("triangle.net partitioning completed. got {0} vertices", voronoi.Vertices.Count);

            List<Line2F> inner_segments = new List<Line2F>();

            foreach (TriangleNet.Geometry.Edge e in voronoi.Edges)
            {
                Line2F seg = new Line2F(voronoi.Vertices[e.P0].X,
                                        voronoi.Vertices[e.P0].Y,
                                        voronoi.Vertices[e.P1].X,
                                        voronoi.Vertices[e.P1].Y);

                if (seg.Length() < double.Epsilon)                                    
                    continue;    // extra small segment, discard                
                if (!topo.Is_line_inside_region(seg, ANALIZE_INNER_INTERSECTIONS, general_tolerance))                                    
                    continue;                
                inner_segments.Add(seg);
            }

            return inner_segments;
        }

        // take 3 well-spaced samples, find the center, check for all points to lay on a circle
        private static bool recognize_perfect_circle(List<Point2F> samples, double tolerance, ref Point2F center)
        {
            if (samples.Count < 4)
                return false;

            Point2F sample0 = samples[0];
            Point2F sample1 = samples[samples.Count / 4];
            Point2F sample2 = samples[samples.Count / 2];

            double xc = 0;
            double yc = 0;

            bool got_center = Geom.Utils.Circle_by_3_pt(sample0.X, sample0.Y, sample1.X, sample1.Y, sample2.X, sample2.Y, ref xc, ref yc);

            if (! got_center)
                return false;

            center = new Point2F(xc, yc);
            double radius = center.DistanceTo(sample0);

            foreach (Point2F pt in samples)
            {
                if (Math.Abs(pt.DistanceTo(center) - radius) > tolerance)
                    return false;
            }

            return true;
        }


        private static Point2F prepare_segments_w_auto_startpoint(Topographer topo, List<Line2F> segments, Segpool pool, double min_dist_to_wall)
        {
            // segments are analyzed for mic radius from both ends. passed segmens are inserted in segpool
            // hashed by one or both endpoints. if endpoint is not hashed, segment wouldn't be followed
            // from that side, preventing formation of bad tree.
            // segments are connected later in a greedy fashion, hopefully forming a medial axis covering whole
            // pocket.
            // start segment is the one with the largest mic

            double max_r = double.MinValue;
            Point2F tree_start = Point2F.Undefined;

            foreach (Line2F seg in segments)
            {
                double r1 = topo.Get_dist_to_wall(seg.p1);
                double r2 = topo.Get_dist_to_wall(seg.p2);

                if (r1 >= min_dist_to_wall)
                {
                    pool.Add(seg, false);
                    if (r1 > max_r)
                    {
                        max_r = r1;
                        tree_start = seg.p1;
                    }
                }
                if (r2 >= min_dist_to_wall)
                {
                    pool.Add(seg, true);
                    if (r2 > max_r)
                    {
                        max_r = r2;
                        tree_start = seg.p2;
                    }
                }
            }

            return tree_start;
        }

        // with the manual startpoint
        private static Point2F prepare_segments_w_manual_starpoint(Topographer topo, List<Line2F> segments, Segpool pool, double min_dist_to_wall, double general_tolerance, Point2F startpoint)
        {
            // same as automatic, but seek the segment with the closest end to startpoint
            if (! topo.Is_line_inside_region(new Line2F(startpoint, startpoint), general_tolerance))
            {
                Logger.warn("startpoint is outside the pocket");
                return Point2F.Undefined;
            }

            if (topo.Get_dist_to_wall(startpoint) < min_dist_to_wall)
            {
                Logger.warn("startpoint radius < tool radius");
                return Point2F.Undefined;
            }

            double min_dist = double.MaxValue;
            Point2F tree_start = Point2F.Undefined;

            foreach (Line2F seg in segments)
            {
                double r1 = topo.Get_dist_to_wall(seg.p1);
                double r2 = topo.Get_dist_to_wall(seg.p2);

                if (r1 >= min_dist_to_wall)
                {
                    pool.Add(seg, false);
                    double dist = startpoint.DistanceTo(seg.p1);
                    if (dist < min_dist && topo.Is_line_inside_region(new Line2F(startpoint, seg.p1), general_tolerance))
                    {
                        min_dist = dist;
                        tree_start = seg.p1;
                    }
                }
                if (r2 >= min_dist_to_wall)
                {
                    pool.Add(seg, true);
                    double dist = startpoint.DistanceTo(seg.p2);
                    if (dist < min_dist && topo.Is_line_inside_region(new Line2F(startpoint, seg.p2), general_tolerance))
                    {
                        min_dist = dist;
                        tree_start = seg.p2;
                    }
                }
            }

            return tree_start;
        }

        private static void build_branch(Medial_branch me, Segpool pool, double min_branch_len)
        {
            Point2F running_end = me.End;
            List<Point2F> followers;

            while (true)
            {
                followers = pool.Pull_follow_points(running_end);

                if (followers.Count != 1)
                    break;

                running_end = followers[0];
                me.Add_point(running_end);         // continuation
            }

            if (followers.Count == 0) return; // end of branch, go out

            foreach (Point2F pt in followers)
            {
                Medial_branch b = me.Spawn_child();
                b.Add_point(running_end);
                b.Add_point(pt);
                build_branch(b, pool, min_branch_len);

                if (b.Deep_distance > min_branch_len) // attach only 'long enough'
                    b.Attach_to_parent();
                else
                    Logger.log("skipping short branch");
            }
            me.Postprocess();
        }

        private static void build_tree(Medial_branch root, Segpool pool, double min_branch_len)
        {
            // this will build tree recursively
            build_branch(root, pool, min_branch_len);
        }

        public static bool Build(Medial_branch root, Topographer topo, List<Point2F> samples, double general_tolerance, Point2F startpoint, double min_dist_to_wall)
        {
            // NOTE: circle is a special case for the medial builder - medial axis is just a single point.
            // worth to make a quick check.
            // circle should be handled in a special way, completely skipping the Voronoi partitioning

            List<Line2F> medial_axis_segments;

            Point2F center = Point2F.Undefined;
            if (! recognize_perfect_circle(samples, general_tolerance, ref center))
            {
                medial_axis_segments = get_medial_axis_segments(topo, samples, general_tolerance);                
            }
            else
            {
                Logger.log("recognized perfect circle");
                medial_axis_segments = new List<Line2F>();
                medial_axis_segments.Add(new Line2F(center, center));
            }

            Logger.log("analyzing segments");

            Segpool pool = new Segpool(medial_axis_segments.Count, general_tolerance);

            Point2F tree_start;
            if (startpoint.IsUndefined)
                tree_start = prepare_segments_w_auto_startpoint(topo, medial_axis_segments, pool, min_dist_to_wall);
            else
                tree_start = prepare_segments_w_manual_starpoint(topo, medial_axis_segments, pool, min_dist_to_wall, general_tolerance, startpoint);

            if (tree_start.IsUndefined)
            {
                Logger.warn("failed to choose tree start point");
                return false;
            }

            Logger.log("done analyzing segments");
            Logger.log("got {0} hashes", pool.N_hashes);

            if (! startpoint.IsUndefined)
                root.Add_point(startpoint);
            root.Add_point(tree_start);

            build_tree(root, pool, general_tolerance);
            return true;
        }
    }
}
