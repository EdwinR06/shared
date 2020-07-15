package org.firstinspires.ftc.teamcode.shared;

import java.util.ArrayList;
import java.util.Objects;

import javax.sound.sampled.Line;


public class Point {
    public double x;
    public double y;

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        Point point = (Point) o;
        return Double.compare(point.x, x) == 0 &&
                Double.compare(point.y, y) == 0;
    }

    @Override
    public int hashCode() {
        return Objects.hash(x, y);
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    @Override
    public String toString() {
        return "Point{" +
                "x=" + x +
                ", y=" + y +
                '}';
    }

    public double distanceFromOrigin() {
        //This is the equation for finding the square root of a^2+b^2. Also, known as pythagorean theorem.
        double distance = Math.sqrt(x * x + y * y);

        return distance;
    }

    public String getQuadrant() {
        //This determines what quadrant a given point is in.
        if (x > 0 && y > 0) {
            return "Quadrant 1";
        } else if (x < 0 && y > 0) {
            return "Quadrant 2";
        } else if (x < 0 && y < 0) {
            return "Quadrant 3";
        } else if (x > 0 && y < 0) {
            return "Quadrant 4";
        } else {
            return "Axes";
        }
    }

    public Point(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public double distanceToPoint(Point other) {
        //This finds the change in x from one point to another.
        double deltaA = x - other.getX();

        //This finds the change in y from one point to another.
        double deltaB = y - other.getY();

        //This performs the Math.sqrt method on the change x and change in y. In addition, it stores that value in the double distance to Point.
        double distanceToPoint = Math.sqrt(deltaA * deltaA + deltaB * deltaB);
        return distanceToPoint;
    }

    public Point closestPoint(Point[] pointsArray) {
        Point closestPoint = pointsArray[0];
        for (int i = 1; i < pointsArray.length; i++) {
            if (this.distanceToPoint(pointsArray[i]) < this.distanceToPoint(closestPoint)) {
                closestPoint = pointsArray[i];
            }
        }
        return closestPoint;
    }

    public static double distanceBetweenTwoPoints(Point a, Point b) {
        double distanceX = a.getX() - b.getX();
        double distanceY = a.getY() - b.getY();

        double distanceBetweenTwoPoint = Math.sqrt(distanceX * distanceX + distanceY * distanceY);
        return distanceBetweenTwoPoint;
    }

    public static Point midPoint(Point a, Point b) {
        double distanceX = a.getX() - b.getX();
        double distanceY = a.getY() - b.getY();

        double distanceBetweenTwoPointForMidPoint = Math.sqrt(distanceX * distanceX + distanceY * distanceY);
        double midPointLength = distanceBetweenTwoPointForMidPoint / 2;
        Point midPoint = new Point(a.getX() + midPointLength, a.getY() + midPointLength);

        return midPoint;
    }

    /*public Point closestWayPoint(ArrayList<Path.WayPoint> closestWayPoint) {
        Point closestWayPoint1 = closestWayPoint.get(0).point;
        for (int i = 1; i < closestWayPoint.size(); i++) {
            if (this.distanceToPoint(closestWayPoint.get(i).point) < this.distanceToPoint(closestWayPoint1)) {
                closestWayPoint1 = closestWayPoint.get(i).point;
            }
        }
        return closestWayPoint1;
    }

    public Point closestPointAlongPath(LineSegment other) {

        return new Point(0, 0);
    }*/




}

