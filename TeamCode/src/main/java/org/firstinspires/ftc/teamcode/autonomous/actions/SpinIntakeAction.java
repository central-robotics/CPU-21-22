package org.firstinspires.ftc.teamcode.autonomous.actions;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
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
        if (!hardware.intakeSpinning) {
            hardware.intakeMotor.setPower(-1);
            hardware.intakeSpinning = true;

            new Thread(() -> {
                double start = System.currentTimeMillis();

                while (System.currentTimeMillis() - start < 3500) {
                    if (hardware.intakeMotor.getCurrent(CurrentUnit.AMPS) > 4.85) {

                        double intakeStart = System.currentTimeMillis();

                        while (System.currentTimeMillis() - intakeStart < 500)
                        {
                            //Do nothing
                        }
                        break;
                    }
                }

                hardware.sweeperServo.setPosition(0.49);

                hardware.intakeMotor.setPower(0.001);

            }).start();
        }
    }
}

