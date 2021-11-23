package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.autonomous.hardware.Hardware;
import org.firstinspires.ftc.teamcode.autonomous.localization.Localization;
import org.firstinspires.ftc.teamcode.autonomous.localization.Position;
import org.firstinspires.ftc.teamcode.autonomous.localization.Velocity;
import org.firstinspires.ftc.teamcode.autonomous.waypoint.Navigation;

public class AutonCore extends LinearOpMode {
    public Hardware hardware;
    public static ElapsedTime runtime;
    public Position robotPosition;
    public Velocity robotVelocity;
    private Localization localization;
    private Navigation navigation;
    private int instruction;

    @Override
    public void runOpMode() {
        hardware = new Hardware(hardwareMap);
        localization = new Localization(hardware);
        navigation = new Navigation(hardware, localization);

        waitForStart();

        runtime.reset();

        while(!isStopRequested())
        {
            robotPosition = localization.getRobotPosition();
            localization.increment(robotPosition);
            robotVelocity = localization.getRobotVelocity(runtime);
        }
    }
}
