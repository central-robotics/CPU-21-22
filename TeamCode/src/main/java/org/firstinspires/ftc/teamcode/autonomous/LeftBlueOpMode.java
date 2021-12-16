package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class LeftBlueOpMode extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Constants.IS_LEFT_OPMODE = true;
        Constants.IS_BLUE_TEAM = true;
        new AutonCore().runCore(Constants.LEFT_INITIAL_X, Constants.INITIAL_Y, this);
    }

}
