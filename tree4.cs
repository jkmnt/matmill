using System;
using System.Collections.Generic;

namespace Tree4
{
    public struct T4_rect
    {
        public double Xmin;
        public double Xmax;
        public double Ymin;
        public double Ymax;

        public double W { get { return Xmax - Xmin; } }
        public double H { get { return Ymax - Ymin; } }

        public double Xc { get { return (Xmax + Xmin) / 2; } }
        public double Yc { get { return (Ymax + Ymin) / 2; } }

        public double Max_squared_distance_to_pt(double x, double y)
        {
            double dx0 = (Xmin - x);
            double dx1 = (Xmax - x);
            double dy0 = (Ymin - y);
            double dy1 = (Ymax - y);

            double d0 = (dx0 * dx0) + (dy0 * dy0);
            double d1 = (dx1 * dx1) + (dy0 * dy0);
            double d2 = (dx0 * dx0) + (dy1 * dy1);
            double d3 = (dx1 * dx1) + (dy1 * dy1);

            return Math.Max(Math.Max(Math.Max(d0, d1), d2), d3);
        }

        public double Min_squared_outside_distance_to_pt(double x, double y)
        {
            double dx = 0;
            double dy = 0;
            if (x < Xmin)
                dx = Xmin - x;
            else if (x > Xmax)
                dx = x - Xmax;

            if (y < Ymin)
                dy = Ymin - y;
            else if (y > Ymax)
                dy = y - Ymax;

            //NOTE: will return 0 for the point inside rect
            return (dx * dx) + (dy * dy);
        }

        public double Max_distance_to_pt(double x, double y)
        {
            return Math.Sqrt(Max_squared_distance_to_pt(x, y));
        }

        public double Min_outside_distance_to_pt(double x, double y)
        {
            return Math.Sqrt(Min_squared_outside_distance_to_pt(x, y));
        }

        public T4_rect(double xmin, double ymin, double xmax, double ymax)
        {
            this.Xmin = xmin;
            this.Xmax = xmax;
            this.Ymin = ymin;
            this.Ymax = ymax;
        }
    }

    public class T4
    {
        private struct T4_occupant
        {
            public T4_rect Rect;
            public object Obj;

            public T4_occupant(T4_rect rect, object obj)
            {
                this.Rect = rect;
                this.Obj = obj;
            }
        }

        public const int MAX_OCCUPANTS = 8;

        private T4[] _rooms;
        private T4_rect _rect;
        private List<T4_occupant> _occupants = new List<T4_occupant>();

        private void split()
        {
            double mid_x = _rect.Xc;
            double mid_y = _rect.Yc;

            _rooms = new T4[4];

            // quads ordering is ccw:
            // 0) left bottom
            // 1) right bottom
            // 2) right top
            // 3) left top

            _rooms[0] = new T4(new T4_rect(_rect.Xmin, _rect.Ymin, _rect.Xc, _rect.Yc));
            _rooms[1] = new T4(new T4_rect(_rect.Xc, _rect.Ymin, _rect.Xmax, _rect.Yc));
            _rooms[2] = new T4(new T4_rect(_rect.Xc, _rect.Yc, _rect.Xmax, _rect.Ymax));
            _rooms[3] = new T4(new T4_rect(_rect.Xmin, _rect.Yc, _rect.Xc, _rect.Ymax));
        }

        private int choose_room(T4_occupant occupant)
        {
            bool is_bottom_quads = occupant.Rect.Ymin > _rect.Ymin && occupant.Rect.Ymax < _rect.Yc;
            bool is_top_quads = occupant.Rect.Ymin > _rect.Yc && occupant.Rect.Ymax < _rect.Ymax;

            bool is_left_quads = occupant.Rect.Xmin > _rect.Xmin && occupant.Rect.Xmax < _rect.Xc;
            bool is_right_quads = occupant.Rect.Xmin > _rect.Xc && occupant.Rect.Xmax < _rect.Xmax;

            if (! (is_top_quads ^ is_bottom_quads)) return -1;
            if (! (is_left_quads ^ is_right_quads)) return -1;

            if (is_bottom_quads)
                return is_left_quads ? 0 : 1;

            return is_right_quads ? 2 : 3;
        }

        private void relocate()
        {
            if (_rooms == null)
                split();

            // try to relocate occupants to the smaller rooms
            for (int i = _occupants.Count - 1; i >= 0; i--)
            {
                T4_occupant occupant = _occupants[i];
                int idx = choose_room(occupant);
                if (idx < 0) continue;  // occupant is too large

                _occupants.RemoveAt(i);
                _rooms[idx].add(occupant);

            }
        }

        private void add(T4_occupant occupant)
        {
            if (_rooms != null)
            {
                int idx = choose_room(occupant);   // choosed a room for occupant successfully
                if (idx != -1)
                {
                    _rooms[idx].add(occupant);
                    return;
                }
            }

            _occupants.Add(occupant);

            if (_occupants.Count > MAX_OCCUPANTS)
                relocate();
        }

        public void Add(T4_rect rect, object obj)
        {
            add(new T4_occupant(rect, obj));
        }

        public List<T4_rect> Traverse_rects()
        {
            List<T4_rect> result = new List<T4_rect>();

            result.Add(_rect);

            if (_rooms != null)
            {
                foreach (T4 room in _rooms)
                {
                    result.AddRange(room.Traverse_rects());
                }
            }

            return result;
        }

        private List<T4_occupant> get_internal_occupants(double x, double y, ref double max_squared_dist)
        {
            List<T4_occupant> result = new List<T4_occupant>();

            if (_rooms != null)
            {
                // if there is a rooms, there are some objects.
                // so we may safely trim max distance by own dimensions
                double max_own_dist = _rect.Max_squared_distance_to_pt(x, y);
                if (max_own_dist < max_squared_dist)
                    max_squared_dist = max_own_dist;

                // sort rooms by minimum distance and visit 'em
                int[] room_idxes = new int[] { 0, 1, 2, 3 };
                double[] room_distances = new double[]
                {
                    _rooms[0]._rect.Min_squared_outside_distance_to_pt(x, y),
                    _rooms[1]._rect.Min_squared_outside_distance_to_pt(x, y),
                    _rooms[2]._rect.Min_squared_outside_distance_to_pt(x, y),
                    _rooms[3]._rect.Min_squared_outside_distance_to_pt(x, y),
                };

                Array.Sort(room_distances, room_idxes);

                // if max distance bound changed while visiting, some rooms would be skipped
                for (int i = 0; i < 4; i++)
                {
                    if (room_distances[i] <= max_squared_dist)
                        result.AddRange(_rooms[room_idxes[i]].get_internal_occupants(x, y, ref max_squared_dist));
                }
            }

            // if there are some objects inside the room, check distance to them
            foreach (T4_occupant occupant in _occupants)
            {
                double min = occupant.Rect.Min_squared_outside_distance_to_pt(x, y);
                if (min > max_squared_dist)
                    continue;   // not in touch

                result.Add(occupant);

                double max = occupant.Rect.Max_squared_distance_to_pt(x, y);
                if (max < max_squared_dist)
                    max_squared_dist = max;
            }

            return result;
        }

        private List<T4_occupant> get_nearest_occupants(double x, double y)
        {
            double max_squared_dist = _rect.H * _rect.H + _rect.W * _rect.W;
            List<T4_occupant> occupants = get_internal_occupants(x, y, ref max_squared_dist);

            // shouldn't happen for non-empty regions and pickpoint inside
            if (occupants.Count == 0) return occupants;

            // purge outdated candidates (collected before dist refinement)
            // find nearest occupant. all chances nearest occupant would be the last added
            // because it trims the max dist most
            // remove all occupants starting after the end of the nearest

            T4_occupant nearest = occupants[occupants.Count - 1];
            double nearest_end = nearest.Rect.Max_squared_distance_to_pt(x, y);

            for (int i = occupants.Count - 2; i >= 0; i--)
            {
                if (occupants[i].Rect.Min_squared_outside_distance_to_pt(x, y) > nearest_end)
                    occupants.RemoveAt(i);
            }

            return occupants;
        }

        public List<object> Get_nearest_objects(double x, double y)
        {
            List<object> objects = new List<object>();

            foreach (T4_occupant occupant in get_nearest_occupants(x, y))            
                objects.Add(occupant.Obj);            

            return objects;
        }

        public List<T4_rect> Get_nearest_obj_rects(double x, double y)
        {
            List<T4_rect> rects = new List<T4_rect>();

            foreach (T4_occupant occupant in get_nearest_occupants(x, y))            
                rects.Add(occupant.Rect);            

            return rects;
        }

        public T4(T4_rect rect)
        {
            _rect = rect;
        }
    }
}