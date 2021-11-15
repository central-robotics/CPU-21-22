package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.autonomous.hardware.HardwareUtil;
import org.firstinspires.ftc.teamcode.autonomous.localization.Localizer;
import org.firstinspires.ftc.teamcode.autonomous.localization.Position;
import org.firstinspires.ftc.teamcode.autonomous.waypoint.Waypoint;

public class AutonCore extends LinearOpMode {

    public static HardwareUtil hardwareUtil;
    public static ElapsedTime runtime;
    public static  Localizer localizer;
    private int instruction;

    @Override
    public void runOpMode() {
        instruction = 0;

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

            Position startingPos = new Position();
            Position targetPos = new Position();

            Waypoint testWaypoint = new Waypoint(startingPos, targetPos, instruction);
        }


    }
}
