package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.autonomous.hardware.HardwareUtil;
import org.firstinspires.ftc.teamcode.autonomous.localization.Localizer;

public class AutonCore extends LinearOpMode {

    public static HardwareUtil hardwareUtil;
    public static ElapsedTime runtime;
    private Localizer localizer;

    @Override
    public void runOpMode() {
        runtime = new ElapsedTime();
        hardwareUtil = new HardwareUtil();
        hardwareUtil.initializeRobot();
        localizer = new Localizer();
        localizer.initializeLocalizer();


        waitForStart();

        runtime.reset();

        while(!isStopRequested())
        {
            localizer.updatePosition();
            telemetry.addData("X", localizer.robotPos.x);
            telemetry.addData("Y", localizer.robotPos.y);
            telemetry.addData("angle", localizer.robotPos.t);
        }


    }
}
