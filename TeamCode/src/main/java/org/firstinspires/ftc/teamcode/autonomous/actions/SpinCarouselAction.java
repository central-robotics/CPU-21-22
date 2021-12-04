package org.firstinspires.ftc.teamcode.autonomous.actions;

import org.firstinspires.ftc.teamcode.autonomous.hardware.Hardware;
import org.firstinspires.ftc.teamcode.autonomous.localization.Localization;

public class SpinCarouselAction extends Action {

    public SpinCarouselAction(int index)
    {
        super(index);
    }

    @Override
    public void execute(Hardware hardware, Localization localization)
    {
        hardware.carouselMotor.setTargetPosition(hardware.carouselMotor.getCurrentPosition() + 1000); //Spin 1000 ticks. We don't care about the exact position, so we'll just add it to our current tick count.
        hardware.carouselMotor.setPower(0.4);
    }

}
