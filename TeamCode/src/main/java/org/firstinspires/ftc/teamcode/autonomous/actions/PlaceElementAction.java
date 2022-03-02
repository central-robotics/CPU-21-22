package org.firstinspires.ftc.teamcode.autonomous.actions;

import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.autonomous.actions.util.ObjectDetector;
import org.firstinspires.ftc.teamcode.autonomous.hardware.Hardware;
import org.firstinspires.ftc.teamcode.autonomous.localization.Localization;
import org.firstinspires.ftc.teamcode.autonomous.vision.Vuforia;
import org.firstinspires.ftc.teamcode.shared.control.PID;

public class PlaceElementAction extends Action{
    public PlaceElementAction(int index) {
        super(index);
    }

    @Override
    public void execute(Hardware hardware, Localization localization, Vuforia vuforia, ObjectDetector detector) {
        PID slidePID = new PID(new PIDCoefficients(0.008, 0.0001, 0));

        int armPos = hardware.armMotor.getCurrentPosition();

        while (armPos < 545) {
            double error = 565 - armPos;
            hardware.armMotor.setPower(0.2 * slidePID.getOutput(error, 0));
            armPos = hardware.armMotor.getCurrentPosition();
        }

        ElapsedTime time = new ElapsedTime();

        hardware.boxServo.setPosition(0);

        time.reset();

        while (time.milliseconds() < 1000)
        {
            //Nothing
        }

        hardware.boxServo.setPosition(0.7);

        armPos = hardware.armMotor.getCurrentPosition();

        while (armPos > 20)
        {
            if (armPos < 100)
                break;

            double error = 20 - armPos;

            hardware.armMotor.setPower(0.2 * slidePID.getOutput(error, 0));

            armPos = hardware.armMotor.getCurrentPosition();
        }
    }
}
