package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class RightRedOpMode extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        new AutonCore().runCore(Constants.RIGHT_INITIAL_X, Constants.INITIAL_Y, this);
    }

}
