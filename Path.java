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


    public ArrayList<WayPoint> getWayPoints() {
        return pathWayPoints;
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
    public WayPoint targetPoint(Point current, double targetDistance) {
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
        if(remainingDistance > 0){
            return pathWayPoints.get(iNext);
        }

        remainingDistance += pathWayPoints.get(iNext).distanceFromPrevious;
        Point firstWayPointForInterpolation = pathWayPoints.get(iNext - 1).point;
        Point secondWayPointForInterpolation = pathWayPoints.get(iNext).point;
        LineSegment ls = new LineSegment(firstWayPointForInterpolation, secondWayPointForInterpolation);
        Point target = ls.interpolate(remainingDistance);

        double distFromStart = remainingDistance + pathWayPoints.get(iNext - 1).distanceFromStart;

        return new WayPoint(target, target.getX() - firstWayPointForInterpolation.getX(), target.getY() - secondWayPointForInterpolation.getY(), remainingDistance, distFromStart);
    }


    /*public WayPoint targetPoint(Point current, double targetDistance) {

        // First, find the next WayPoint along the path
        int nextWayPoint = findNextWayPoint(current);

        // Add up distances to find a WayPoint just more than distance away from current
        WayPoint targetWayPoint;
        double distanceSoFar = pathWayPoints.get(nextWayPoint).componentAlongPath(current);
        int found = nextWayPoint - 1;
        while (distanceSoFar < targetDistance && found < pathWayPoints.size() - 1) {
            distanceSoFar += pathWayPoints.get(found).distanceFromPrevious;
            found++;
        }

        // Interpolate between that Waypoint and the one before
        if (distanceSoFar < targetDistance) {
            //We ran off the end of the path before exceeding the target distance
            targetWayPoint = pathWayPoints.get(found);
        } else {
            LineSegment interpolateLineSegment = new LineSegment(pathWayPoints.get(found - 1).point, pathWayPoints.get(found).point);
            Point targetPoint = interpolateLineSegment.interpolate(targetDistance - distanceSoFar);
            targetWayPoint = new WayPoint(targetPoint, pathWayPoints.get(found).point.getX() - pathWayPoints.get(found - 1).point.getX(), pathWayPoints.get(found).point.getY() - pathWayPoints.get(found - 1).point.getY(), Point.distanceBetweenTwoPoints(pathWayPoints.get(found).point, pathWayPoints.get(found - 1).point));

        }
        return targetWayPoint;





        /*double distanceLeft;

        boolean isCurrentPointAWayPoint;
        WayPoint endOfPath = pathWayPoints.get(pathWayPoints.size() - 1);

        for (int i = 0; i < pathWayPoints.size(); i++) {
            if (current.equals(pathWayPoints.get(i).point)) {
                isCurrentPointAWayPoint = true;
            } else {
                isCurrentPointAWayPoint = false;
            }
        }
        Point newestStartingPoint;
        Point nextTargetPoint;
        LineSegment interpolatedLineSegmentForStartingPoint;
        LineSegment newSegment;
        WayPoint newTargetPointFromCurrentPoint = new WayPoint(new Point(0, 0), 0, 0, 0);
        WayPoint newTargetWayPointFromCurrentWayPoint = new WayPoint(new Point(0, 0), 0, 0, 0);
        WayPoint targetWayPoint = new WayPoint(new Point(0, 0), 0, 0, 0);
        WayPoint newTargetFartherAlongPath = new WayPoint(new Point(0, 0), 0, 0, 0);
        double startingWayPoint;
        // WayPoint
        if (isCurrentPointAWayPoint = true) {
            for (int i = 0; i < pathWayPoints.size(); i++) {
                if (current.equals(pathWayPoints.get(i).point)) {

                    LineSegment pathSegment = new LineSegment(current, pathWayPoints.get(i + 1).point);

                    if (targetDistance > pathSegment.length) {

                        double nextLineSegmentDistance = -1 * (pathSegment.length - targetDistance);

                        Point partialInterpolationToTargetPoint = new Point(pathWayPoints.get(i + 1).point.getX(), pathWayPoints.get(i + 1).point.getY());

                        pathSegment = new LineSegment(partialInterpolationToTargetPoint, pathWayPoints.get(i + 2).point);

                        boolean whatI = false;

                        while (nextLineSegmentDistance > pathSegment.length) {

                            nextLineSegmentDistance = -1 * (pathSegment.length - targetDistance);

                            partialInterpolationToTargetPoint = new Point(pathWayPoints.get(i + 2).point.getX(), pathWayPoints.get(i + 2).point.getY());

                            pathSegment = new LineSegment(partialInterpolationToTargetPoint, pathWayPoints.get(i + 3).point);


                        }

                        Point targetPointFromPreviousWayPoint = pathSegment.interpolate(nextLineSegmentDistance);

                        if (whatI = true) {
                            targetWayPoint = new WayPoint(targetPointFromPreviousWayPoint, pathWayPoints.get(i + 3).point.getX() - partialInterpolationToTargetPoint.getX(), pathWayPoints.get(i + 3).point.getY() - partialInterpolationToTargetPoint.getY(), nextLineSegmentDistance);

                        } else {
                            targetWayPoint = new WayPoint(targetPointFromPreviousWayPoint, pathWayPoints.get(i + 3).point.getX() - partialInterpolationToTargetPoint.getX(), pathWayPoints.get(i + 3).point.getY() - partialInterpolationToTargetPoint.getY(), nextLineSegmentDistance);
                        }

                    } else {
                        Point newTargetPointFromCurrentWayPoint = pathSegment.interpolate(targetDistance);

                        targetWayPoint = new WayPoint(newTargetPointFromCurrentWayPoint, pathWayPoints.get(i + 1).point.getX() - current.x, pathWayPoints.get(i + 1).point.getY() - current.y, targetDistance);
                    }
                }
            }
        } else {
            int j = 0;
            do {
                j++;
                continue;
            } while (pathWayPoints.get(j).componentAlongPath(current) < 0);
            interpolatedLineSegmentForStartingPoint = new LineSegment(pathWayPoints.get(j).point, pathWayPoints.get(j - 1).point);
            newestStartingPoint = interpolatedLineSegmentForStartingPoint.interpolate(-1 * (pathWayPoints.get(j).componentAlongPath(current)));
            newSegment = new LineSegment(newestStartingPoint, pathWayPoints.get(j).point);
            if (targetDistance > newSegment.length) {

                double nextLineSegmentDistance = -1 * (newSegment.length - targetDistance);

                Point partialInterpolationToTargetPoint = new Point(pathWayPoints.get(j + 1).point.getX(), pathWayPoints.get(j + 1).point.getY());

                newSegment = new LineSegment(partialInterpolationToTargetPoint, pathWayPoints.get(j + 2).point);

                boolean whatI = false;

                while (nextLineSegmentDistance > newSegment.length) {

                    nextLineSegmentDistance = -1 * (newSegment.length - targetDistance);

                    partialInterpolationToTargetPoint = new Point(pathWayPoints.get(j + 2).point.getX(), pathWayPoints.get(j + 2).point.getY());

                    newSegment = new LineSegment(partialInterpolationToTargetPoint, pathWayPoints.get(j + 3).point);


                }

                Point targetPointFromPreviousWayPoint = newSegment.interpolate(nextLineSegmentDistance);

                if (whatI = true) {
                    targetWayPoint = new WayPoint(targetPointFromPreviousWayPoint, pathWayPoints.get(j + 3).point.getX() - partialInterpolationToTargetPoint.getX(), pathWayPoints.get(j + 3).point.getY() - partialInterpolationToTargetPoint.getY(), nextLineSegmentDistance);

                } else {
                    targetWayPoint = new WayPoint(targetPointFromPreviousWayPoint, pathWayPoints.get(j + 3).point.getX() - partialInterpolationToTargetPoint.getX(), pathWayPoints.get(j + 3).point.getY() - partialInterpolationToTargetPoint.getY(), nextLineSegmentDistance);
                }

            }
        }
        if (current == endOfPath.point || ((current.getX() > endOfPath.point.getX()) && (current.getY() > endOfPath.point.getY()))) {
            return endOfPath;
        }
        return targetWayPoint;*/

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


    public static class WayPoint {
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
