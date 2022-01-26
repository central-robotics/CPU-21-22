package org.firstinspires.ftc.teamcode.autonomous.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.autonomous.AutonCore;
import org.firstinspires.ftc.teamcode.autonomous.Constants;

@Autonomous
public class RightRedOpMode extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        new AutonCore().runCore(Constants.INITIAL_X, Constants.RIGHT_INITIAL_Y, Constants.RED_INITIAL_THETA, this);
    }

}
