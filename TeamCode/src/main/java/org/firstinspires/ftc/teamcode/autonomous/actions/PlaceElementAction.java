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

        hardware.boxServo.setPosition(0.68);

        while (armPos < 440) {
            double error = 440 - armPos;
            hardware.armMotor.setPower(0.2 * slidePID.getOutput(error, 0));
            armPos = hardware.armMotor.getCurrentPosition();


            if (armPos > 50) {
                {
                    hardware.boxServo.setPosition(0.35);
                }
            }
        }

        ElapsedTime time = new ElapsedTime();

        hardware.boxServo.setPosition(0);

        time.reset();

        while (time.milliseconds() < 1500) {
            //Nothing
        }

        hardware.boxServo.setPosition(0.68);

        while (time.milliseconds() < 1500) {
            //Nothing
        }

        armPos = hardware.armMotor.getCurrentPosition();

        while (armPos > 20) {
            if (armPos < 100)
                break;

            double error = 20 - armPos;

            hardware.armMotor.setPower(0.2 * slidePID.getOutput(error, 0));

            armPos = hardware.armMotor.getCurrentPosition();
        }
    }
}
