package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.autonomous.hardware.Hardware;
import org.firstinspires.ftc.teamcode.autonomous.localization.Localization;
import org.firstinspires.ftc.teamcode.autonomous.localization.Position;
import org.firstinspires.ftc.teamcode.autonomous.waypoint.Navigation;
import org.firstinspires.ftc.teamcode.autonomous.waypoint.Waypoint;

public class AutonCore extends LinearOpMode {
    public Hardware hardware;
    public static ElapsedTime runtime;
    public Position robotPosition;
    private Localization localization;
    private Navigation navigation;
    private int instruction;

    @Override
    public void runOpMode() {
        hardware = new Hardware(hardwareMap);
        localization = new Localization(hardware);
        navigation = new Navigation(hardware, localization);
        robotPosition = localization.getRobotPosition();

        waitForStart();

        runtime.reset();

        while(!isStopRequested())
        {

        }
    }
}
