package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.autonomous.hardware.HardwareUtil;

public class AutonCore extends LinearOpMode {

    HardwareUtil hardwareUtil;

    @Override
    public void runOpMode() {
        hardwareUtil = new HardwareUtil();
        hardwareUtil.InitializeRobot();

        waitForStart();
    }
}
