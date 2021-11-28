package org.firstinspires.ftc.teamcode.autonomous.control;

import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.teamcode.autonomous.localization.Position;
import org.firstinspires.ftc.teamcode.autonomous.localization.Velocity;

public class Controller {
    PID posController;
    PID negController;
    RelativeRobotPos out;

    public Controller(PIDCoefficients coeffs)
    {
        posController = new PID(coeffs);
        negController = new PID(coeffs);
        out = new RelativeRobotPos();
    }

    public RelativeRobotPos eval(Position pos, Position target, Velocity vel)
    {
        out.pos = posController.eval(target.positive, pos.positive, vel.dx);
        out.neg = negController.eval(target.negative, pos.negative, vel.dy);

        return out;
    }
}
