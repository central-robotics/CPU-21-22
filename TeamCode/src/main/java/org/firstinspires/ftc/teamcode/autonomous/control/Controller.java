package org.firstinspires.ftc.teamcode.autonomous.control;

import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.teamcode.autonomous.localization.Position;
import org.firstinspires.ftc.teamcode.autonomous.localization.Velocity;

public class Controller {
    PID xController;
    PID yController;
    PID tController;
    ControllerOutput out;

    public Controller(PIDCoefficients coeffs)
    {
        xController = new PID(coeffs);
        yController = new PID(coeffs);
        tController = new PID(coeffs);
        out = new ControllerOutput();
    }

    public ControllerOutput eval(Position pos, Position target, Velocity vel)
    {
        out.xOut = xController.eval(target.x, pos.x, vel.dx);
        out.yOut = yController.eval(target.y, pos.y, vel.dy);
        out.tOut = tController.eval(target.t, pos.t, vel.dt);

        return out;
    }
}
