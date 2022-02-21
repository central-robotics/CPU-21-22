package org.firstinspires.ftc.teamcode.autonomous.actions;

import org.firstinspires.ftc.teamcode.autonomous.Constants;
import org.firstinspires.ftc.teamcode.autonomous.actions.util.ObjectDetector;
import org.firstinspires.ftc.teamcode.autonomous.hardware.Hardware;
import org.firstinspires.ftc.teamcode.autonomous.instructions.Instructions;
import org.firstinspires.ftc.teamcode.autonomous.localization.Localization;
import org.firstinspires.ftc.teamcode.autonomous.localization.Position;
import org.firstinspires.ftc.teamcode.autonomous.nav.Navigation;
import org.firstinspires.ftc.teamcode.autonomous.vision.Vuforia;

public class SpinCarouselAction extends Action {

    public SpinCarouselAction(int index)
    {
        super(index);
    }

    @Override
    public void execute(Hardware hardware, Localization localization, Vuforia vuforia, ObjectDetector detector)
    {
        long time = System.currentTimeMillis();

        if (!Constants.IS_BLUE_TEAM)
            new Thread(() -> {
                hardware.carouselMotor.setPower(0.4);
            }).start();

        else {
            new Thread(() -> {
                hardware.carouselMotor.setPower(-0.4);
            }).start();
        }

        Position pos = localization.getRobotPosition();
        localization.increment(pos);


        while(System.currentTimeMillis() - time < 4000)
        {
        }

        hardware.carouselMotor.setPower(0);
    }
}
