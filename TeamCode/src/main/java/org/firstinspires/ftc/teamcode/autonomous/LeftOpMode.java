package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class LeftOpMode extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        new AutonCore().runCore(Constants.FIELD_LENGTH / 2 - Constants.OFFSET_FROM_CENTER, this);
    }

}
