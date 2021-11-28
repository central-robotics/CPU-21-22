package org.firstinspires.ftc.teamcode.autonomous.localization;

public class Position {

    public double x, y, t, positive, negative;

    public Position() {};
    public Position(double x, double y, double t, double positive, double negative)
    {
        this.x = x;
        this.y = y;
        this.t = t;
        this.positive = positive;
        this.negative = negative;
    }
}
