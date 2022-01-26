package org.firstinspires.ftc.teamcode.autonomous.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.autonomous.AutonCore;
import org.firstinspires.ftc.teamcode.autonomous.Constants;

@Autonomous
public class RightBlueOpMode extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Constants.IS_BLUE_TEAM = true;
        new AutonCore().runCore(Constants.INITIAL_X, Constants.RIGHT_INITIAL_Y, Constants.BLUE_INITIAL_THETA, this);
    }

}
