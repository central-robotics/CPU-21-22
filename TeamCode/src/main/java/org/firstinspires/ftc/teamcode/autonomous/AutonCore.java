package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.autonomous.hardware.HardwareUtil;
import org.firstinspires.ftc.teamcode.autonomous.localization.Localizer;

public class AutonCore extends LinearOpMode {

    public static HardwareUtil hardwareUtil;
    public static Localizer.Location robotLocation;
    private Localizer localizer;

    @Override
    public void runOpMode() {
        hardwareUtil = new HardwareUtil();
        hardwareUtil.initializeRobot();
        localizer = new Localizer();
        localizer.initializeLocalizer();


        waitForStart();

        while(!isStopRequested())
        {
            robotLocation = localizer.getRobotPosition();


        }


    }
}
