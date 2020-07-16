package org.firstinspires.ftc.teamcode.shared;

public class Vector {
    public double x, y;

    public Vector(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public static Vector makeUnitVector(double direction){
        double x = Math.cos(direction);
        double y = Math.sin(direction);
        return new Vector(x, y);
    }

    public Vector scale(double scale){
        y = y * scale;
        x = x * scale;
        return this;
    }
}
