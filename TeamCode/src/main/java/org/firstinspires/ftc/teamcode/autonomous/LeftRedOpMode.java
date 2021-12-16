package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class LeftRedOpMode extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Constants.IS_LEFT_OPMODE = true;
        //new AutonCore().runCore(0, 0, this);
        new AutonCore().runCore(Constants.LEFT_INITIAL_X, Constants.INITIAL_Y, this);
    }

}
