package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.teamcode.Auton;
import org.firstinspires.ftc.teamcode.autonomous.localization.Localizer;
import org.firstinspires.ftc.teamcode.autonomous.localization.Position;

public class Controller extends AutonCore
{
    // Tune values here; these apply to all three controllers
    PIDCoefficients coeffs = new PIDCoefficients(0, 0, 0);

    PID xController = new PID(coeffs);
    PID yController = new PID(coeffs);
    PID tController = new PID(coeffs);

    // This is basically just pseudocode, I don't expect it to work in any capacity
    public void increment(Position position)
    {
        xController.eval(target, position.x, speed);
        yController.eval(target, position.x, speed);
        tController.eval(target, position.x, speed);
    }
}