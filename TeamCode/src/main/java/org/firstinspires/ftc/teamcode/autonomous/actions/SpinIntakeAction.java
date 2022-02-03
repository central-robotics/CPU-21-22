package org.firstinspires.ftc.teamcode.autonomous.actions;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.autonomous.actions.util.ObjectDetector;
import org.firstinspires.ftc.teamcode.autonomous.hardware.Hardware;
import org.firstinspires.ftc.teamcode.autonomous.localization.Localization;
import org.firstinspires.ftc.teamcode.autonomous.vision.Vuforia;

public class SpinIntakeAction extends Action{

    public SpinIntakeAction(int index) {
        super(index);
    }

    @Override
    public void execute(Hardware hardware, Localization localization, Vuforia vuforia, ObjectDetector detector) {
        if (!hardware.intakeSpinning)
        {
            hardware.intakeMotor.setPower(-1);
            hardware.intakeSpinning = true;
        } else {
            hardware.intakeMotor.setPower(0.001);
            hardware.intakeSpinning = false;
        }
    }
}

