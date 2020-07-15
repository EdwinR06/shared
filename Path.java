package org.firstinspires.ftc.teamcode.shared;

import java.util.ArrayList;
import java.util.List;

public class Path {
    /**
     * @param rawPoints Array of X,Y points.  Duplicate points are discarded
     * A path must have at least 2 non-identical points
     */
    private ArrayList<WayPoint> pathWayPoints = new ArrayList<>();
    List<WayPoint> wayPoints;
    double distanceOfPath = 0;

    public Path(Point[] rawPoints) {

        if (rawPoints.length < 2) {
            throw new IllegalArgumentException("Tried to create a path with too few points.");
        }
        distanceOfPath += totalDistanceOfPath(rawPoints);
        double distanceToStart = 0;

        wayPoints = new ArrayList<>();


        WayPoint firstWayPoint = new WayPoint(rawPoints[0], 0, 0, 0, 0);
        pathWayPoints.add(firstWayPoint);
        for (int i = 1; i < rawPoints.length; i++) {
            double distanceFromPreviousWayPoint = rawPoints[i].distanceToPoint(rawPoints[i - 1]);
            distanceToStart += distanceFromPreviousWayPoint;

            if (!rawPoints[i].equals(rawPoints[i - 1])) {
                pathWayPoints.add(new WayPoint(rawPoints[i], rawPoints[i].getX() - rawPoints[i - 1].getX(), rawPoints[i].getY() - rawPoints[i - 1].getY(), rawPoints[i].distanceToPoint(rawPoints[i - 1]), distanceToStart));
            }
        }
        if (pathWayPoints.size() < 2) {
            throw new IllegalArgumentException("Path must have two unique points.");
        }
    }

    List<TargetPoint> getPoints() {
        List<TargetPoint> points = new ArrayList<>();
        for (WayPoint w : pathWayPoints) {
            points.add(new TargetPoint(w));
        }
        return points;
    }

    public double totalDistanceOfPath(Point[] rawPoints) {
        double totalDistanceOfPath = 0;
        for (int i = 0; i < rawPoints.length - 1; i++) {
            totalDistanceOfPath += Point.distanceBetweenTwoPoints(rawPoints[i], rawPoints[i + 1]);
        }
        return totalDistanceOfPath;
    }


    /**
     * @return total distance of the path
     */
    public double totalDistance() {
        double totalDistanceBetweenTwoWayPoints = 0;
        for (int i = 0; i < pathWayPoints.size() - 1; i++) {

            totalDistanceBetweenTwoWayPoints += Point.distanceBetweenTwoPoints(pathWayPoints.get(i).point, pathWayPoints.get(i + 1).point);


        }
        return totalDistanceBetweenTwoWayPoints;
    }

    /**
     * @return a point at the supplied distance along the path from the supplied current position
     * Note that the point will usually be interpolated between the points that originally defined the Path
     */
    public TargetPoint targetPoint(Point current, double targetDistance) {
        int iNext = 1;
        //Find the next WayPoint ahead of current point
        while (pathWayPoints.get(iNext).componentAlongPath(current) <= 0 && iNext < pathWayPoints.size() - 1) {
            iNext++;

        }
        double remainingDistance;
        remainingDistance = targetDistance - pathWayPoints.get(iNext).componentAlongPath(current);
        while (remainingDistance > 0 && iNext < pathWayPoints.size() - 1) {
            iNext++;
            remainingDistance -= pathWayPoints.get(iNext).distanceFromPrevious;

        }
        //Check to see if we run out of WayPoints before target distance
        if (remainingDistance > 0) {
            return new TargetPoint(pathWayPoints.get(iNext));
        }

        remainingDistance += pathWayPoints.get(iNext).distanceFromPrevious;
        Point firstWayPointForInterpolation = pathWayPoints.get(iNext - 1).point;
        Point secondWayPointForInterpolation = pathWayPoints.get(iNext).point;
        LineSegment ls = new LineSegment(firstWayPointForInterpolation, secondWayPointForInterpolation);
        Point target = ls.interpolate(remainingDistance);

        double distFromStart = remainingDistance + pathWayPoints.get(iNext - 1).distanceFromStart;

        return new TargetPoint(target, distFromStart, distanceFromEnd(distFromStart));
    }

    public boolean isComplete(Point current) {
        WayPoint lastWayPoint = pathWayPoints.get(pathWayPoints.size() - 1);
        return lastWayPoint.componentAlongPath(current) <= 0;
    }

    public double distanceFromEnd(double distanceToStart) {
        double distanceToEnd = 0;


        distanceToEnd += distanceOfPath - distanceToStart;

        return distanceToEnd;
    }

    /**
     * Find the index of the WayPoint that is just ahead of current on the path
     *
     * @param current
     * @return
     */
    private int findNextWayPoint(Point current) {
        int iNextWayPoint = 0;
        for (int i = 0; i < pathWayPoints.size(); i++) {
            if (pathWayPoints.get(i).componentAlongPath(current) > 0) {
                iNextWayPoint = i;
                break;
            }
        }
        return iNextWayPoint;
    }

    public static class TargetPoint {
        public Point point;
        public double distanceFromStart;
        public double distanceToEnd;

        public TargetPoint(Point point, double distanceFromStart, double distanceFromEnd) {
            this.point = point;
            this.distanceFromStart = distanceFromStart;
            this.distanceToEnd = distanceFromEnd;
        }

        public TargetPoint(WayPoint wayPoint) {
            this.point = wayPoint.point;
            this.distanceFromStart = wayPoint.distanceFromStart;
            this.distanceToEnd = wayPoint.getDistanceToEnd();
        }
    }

    private class WayPoint {
        public Point point;
        public double deltaXFromPrevious;
        public double deltaYFromPrevious;
        public double distanceFromPrevious;
        public double distanceFromStart;


        public WayPoint(Point point, double deltaXFromPrevious, double deltaYFromPrevious, double distanceFromPrevious, double distanceFromStart) {
            this.point = point;
            this.deltaXFromPrevious = deltaXFromPrevious;
            this.deltaYFromPrevious = deltaYFromPrevious;
            this.distanceFromPrevious = distanceFromPrevious;
            this.distanceFromStart = distanceFromStart;

        }

        public double getDistanceToEnd() {
            return totalDistance() - distanceFromStart;
        }

        /**
         * Calculates the projection of the vector Vcurrent leading from the supplied current
         * point to this WayPoint onto the vector Vpath leading from the previous point on the path
         * to this WayPoint.  If the return value is positive, it means that the WayPoint is
         * farther along the path from the current point.  If the return value is negative, it means
         * that the WayPoint is before the current point.  The magnitude of the value tells the
         * distance along the path.  The value is computed as the dot product between Vcurrent and
         * Vpath, normalized by the length of vPath
         *
         * @param current The source point to compare to the WayPoint
         */
        private double componentAlongPath(Point current) {
            double deltaXFromCurrent = point.x - current.x;
            double deltaYFromCurrent = point.y - current.y;

            double dp = deltaXFromCurrent * deltaXFromPrevious + deltaYFromCurrent * deltaYFromPrevious;
            double projection = dp / distanceFromPrevious;

            return projection;
        }
    }
}
