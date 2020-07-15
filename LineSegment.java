package org.firstinspires.ftc.teamcode.shared;

import java.util.ArrayList;

public class LineSegment {
    //These  points are created to use within methods as place holders for the points called in tests and prints.
    private Point point1;
    private Point point2;
    public double length;

    public LineSegment(Point point1, Point point2) {
        this.point1 = point1;
        this.point2 = point2;
        this.length = Point.distanceBetweenTwoPoints(point1, point2);
    }
    // The Point[] below means that the method will return an array of points.
    public Point[] subDivide(int subSegments) {
        //Find the deltas of the two x coordinates of the line segment end points.
        double deltaX = point2.getX() - point1.getX();
        //Find the deltas of the two y coordinates of the line segment end points.
        double deltaY = point2.getY() - point1.getY();

        //Find the x step for each segment.
        double xStep = deltaX / subSegments;
        //Find the y step for each segment.
        double yStep = deltaY / subSegments;
        Point[] subDividedPoints = new Point[subSegments - 1];
        for(int i = 1; i < subSegments; i++){
            subDividedPoints[i-1] = new Point (point1.getX() + xStep * i, point1.getY() + yStep * i);
        }
        return subDividedPoints;

    }

    public Point interpolate(double distanceFromFirstPoint){
        double lineSegmentRatio = distanceFromFirstPoint / length;

        Point interpolate = new Point(point1.getX() + (lineSegmentRatio * (point2.getX() - point1.getX())), point1.getY() + (lineSegmentRatio * (point2.getY() - point1.getY())));

        // double xValue = point1.getX() + (lineSegmentRatio *  (point2.getX() - point)

        return interpolate;
    }

}

