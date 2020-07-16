package org.firstinspires.ftc.teamcode.shared;

public class PowerPoint extends Point{
    public double power;

    public PowerPoint(double x, double y, double power) {
        super(x, y);
        this.power = power;
    }

    public Point makePoint(){
        Point point = new Point(x, y);
        return point;
    }
}
