package org.firstinspires.ftc.teamcode.autonomous.control;

import com.qualcomm.robotcore.hardware.PIDCoefficients;

public class PID {
    double kP, kI, kD;

    public PID(PIDCoefficients coeffs)
    {
        kP = coeffs.p;
        kI = coeffs.i;
        kD = coeffs.d;
    }

    double error, errorSum, speed;

    public double eval(double target, double pos, double _speed)
    {
        error = target - pos;
        errorSum += error;
        speed = _speed;

        return (kP * error) + (kI * errorSum) - (kD * speed);
    }
}
