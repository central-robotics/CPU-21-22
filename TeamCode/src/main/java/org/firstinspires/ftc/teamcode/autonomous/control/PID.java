package org.firstinspires.ftc.teamcode.autonomous.control;

import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.teamcode.autonomous.localization.Position;
import org.firstinspires.ftc.teamcode.autonomous.localization.Velocity;

public class PID {
    private final double kP;
    private final double kI;
    private final double kD;
    private double errorSum;

    public PID(PIDCoefficients coeffs)
    {
        kP = coeffs.p;
        kI = coeffs.i;
        kD = coeffs.d;
    }

    public double getMagnitude(Position target, Position position, Velocity _speed)
    {
        double error = Math.sqrt(Math.pow(target.y - position.y, 2) + Math.pow(target.x - position.x, 2));
        errorSum += error;
        double speed = Math.sqrt(Math.pow(_speed.dy, 2) + Math.pow(_speed.dx, 2));

        return (kP * error) + (kI * errorSum) - (kD * speed);
    }

    public double getSlope(Position target, Position position)
    {
        return (target.y - position.y) / (target.x - position.x);
    }
}
