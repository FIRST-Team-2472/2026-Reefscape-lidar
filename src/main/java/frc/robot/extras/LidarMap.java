package frc.robot.extras;

import frc.robot.extras.LidarMapComponents.*;

public class LidarMap {
    public static Polygon[] map = new Polygon[3];

    public LidarMap() {
        //all numbers are magic so far
        // Define polygons and add them to the map array
        // Polygon 1 aka the field walls 
        //currently just a 10x10 square
    }
    public static Polygon[] getMap() {
        map[0] = new Polygon(new LineSegment[] {
                new LineSegment(
                        new MapPoint(0, 0),
                        new MapPoint(10, 0)),
                new LineSegment(
                        new MapPoint(10, 0),
                        new MapPoint(10, 10)),
                new LineSegment(
                        new MapPoint(10, 10),
                        new MapPoint(0, 10)),
                new LineSegment(
                        new MapPoint(0, 10),
                        new MapPoint(0, 0))
        });

        // Polygon aka hexagon in the middle
        map[1] = new Polygon(new LineSegment[] {
                new LineSegment(
                        new MapPoint(6.000, 5.577),
                        new MapPoint(5.000, 6.155)),
                new LineSegment(
                        new MapPoint(5.000, 6.155),
                        new MapPoint(4.000, 5.577)),
                new LineSegment(
                        new MapPoint(4.000, 5.577),
                        new MapPoint(4.000, 4.423)),
                new LineSegment(
                        new MapPoint(4.000, 4.423),
                        new MapPoint(5.000, 3.845)),
                new LineSegment(
                        new MapPoint(5.000, 3.845),
                        new MapPoint(6.000, 4.423)),
                new LineSegment(
                        new MapPoint(6.000, 4.423),
                        new MapPoint(6.000, 5.577))
        });
        // a rectangle with dimensions of a Tower upright in REBUILT FRC field, centered at the origin
        map[2] = new Polygon(
                new LineSegment[] {
                        new LineSegment(
                                new MapPoint(-0.01905, 0.04445),
                                new MapPoint(0.01905, 0.04445)),
                        new LineSegment(
                                new MapPoint(0.01905, 0.04445),
                                new MapPoint(0.01905, -0.04445)),
                        new LineSegment(
                                new MapPoint(0.01905, -0.04445),
                                new MapPoint(-0.01905, -0.04445)),
                        new LineSegment(
                                new MapPoint(-0.01905, -0.04445),
                                new MapPoint(-0.01905, 0.04445))
                }
        );
        return map;
    }
}