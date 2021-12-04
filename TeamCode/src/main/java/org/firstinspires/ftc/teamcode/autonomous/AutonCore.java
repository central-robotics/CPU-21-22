package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.autonomous.actions.Actions;
import org.firstinspires.ftc.teamcode.autonomous.hardware.Hardware;
import org.firstinspires.ftc.teamcode.autonomous.localization.Localization;
import org.firstinspires.ftc.teamcode.autonomous.localization.Position;
import org.firstinspires.ftc.teamcode.autonomous.localization.Velocity;
import org.firstinspires.ftc.teamcode.autonomous.waypoint.Navigation;
import org.firstinspires.ftc.teamcode.autonomous.waypoint.Waypoint;

@Autonomous
public class AutonCore extends LinearOpMode {
    public static ElapsedTime runtime;

    @Override
    public void runOpMode() {
        runtime = new ElapsedTime();
        Hardware hardware = new Hardware(hardwareMap);
        Localization localization = new Localization(hardware, this.telemetry);
        Instructions instructions = new Instructions(hardware, localization, runtime, telemetry);

        waitForStart();

        runtime.reset();

        while (runtime.milliseconds() < 500)
        {
            Constants.INIT_THETA = hardware.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS).secondAngle;
        }

        runtime.reset();

        instructions.runTasks();

        instructions.dispose();

        stop();
    }
}
