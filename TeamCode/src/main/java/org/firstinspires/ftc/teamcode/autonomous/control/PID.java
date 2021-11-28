package org.firstinspires.ftc.teamcode.autonomous.control;

import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.teamcode.autonomous.localization.Position;
import org.firstinspires.ftc.teamcode.autonomous.localization.Velocity;

public class PID {
    double kP, kI, kD;

    public PID(PIDCoefficients coeffs)
    {
        kP = coeffs.p;
        kI = coeffs.i;
        kD = coeffs.d;
    }

    double error, errorSum, speed;

    public double eval(Position target, Position position, Velocity _speed)
    {
        error = Math.sqrt(Math.pow(target.y - position.y, 2) + Math.pow(target.x - position.x, 2));
        errorSum += error;
        speed = Math.sqrt(Math.pow(_speed.dy, 2) + Math.pow(_speed.dx, 2));

        return (kP * error) + (kI * errorSum) - (kD * speed);
    }
}
