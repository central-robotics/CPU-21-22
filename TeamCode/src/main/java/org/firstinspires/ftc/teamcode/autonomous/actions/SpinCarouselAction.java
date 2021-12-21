package org.firstinspires.ftc.teamcode.autonomous.actions;

import org.firstinspires.ftc.teamcode.autonomous.Constants;
import org.firstinspires.ftc.teamcode.autonomous.hardware.Hardware;
import org.firstinspires.ftc.teamcode.autonomous.localization.Localization;
import org.firstinspires.ftc.teamcode.autonomous.vision.Vuforia;

public class SpinCarouselAction extends Action {

    public SpinCarouselAction(int index)
    {
        super(index);
    }

    @Override
    public void execute(Hardware hardware, Localization localization, Vuforia vuforia)
    {
        long time = System.currentTimeMillis();
        if (!Constants.IS_BLUE_TEAM)
            hardware.carouselMotor.setPower(-0.4);
        else
            hardware.carouselMotor.setPower(0.4);


        while(System.currentTimeMillis() - time < 4000)
        {
        }
        hardware.carouselMotor.setPower(0);
    }
}
