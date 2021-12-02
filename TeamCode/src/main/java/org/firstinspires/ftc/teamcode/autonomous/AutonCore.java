package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.autonomous.hardware.Hardware;
import org.firstinspires.ftc.teamcode.autonomous.localization.Localization;
import org.firstinspires.ftc.teamcode.autonomous.localization.Position;
import org.firstinspires.ftc.teamcode.autonomous.localization.Velocity;
import org.firstinspires.ftc.teamcode.autonomous.waypoint.Navigation;
import org.firstinspires.ftc.teamcode.autonomous.waypoint.Waypoint;

@Autonomous
public class AutonCore extends LinearOpMode {
    public Hardware hardware;
    public static ElapsedTime runtime;
    private Localization localization;
    private Navigation navigation;

    @Override
    public void runOpMode() {
        hardware = new Hardware(hardwareMap);
        localization = new Localization(hardware, this.telemetry);
        runtime = new ElapsedTime();
        navigation = new Navigation(hardware, localization, runtime, telemetry);

        Position position = new Position();

        waitForStart();

        runtime.reset();

        navigation.addWayPointToQueue(new Waypoint(new Position(0,0,0), new Position(-400, 0,0)));
        navigation.executeTask();

/*        while(!isStopRequested()) {
            position = localization.getRobotPosition();
            localization.increment(position);

            telemetry.addData("X", position.x);
            telemetry.addData("Y", position.y);
            telemetry.addData("T", position.t);
            telemetry.update();
        }*/

        stop();
    }
}
