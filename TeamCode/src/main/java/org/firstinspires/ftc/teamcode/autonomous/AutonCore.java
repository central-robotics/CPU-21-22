package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.autonomous.vision.Vuforia;
import org.firstinspires.ftc.teamcode.autonomous.hardware.Hardware;
import org.firstinspires.ftc.teamcode.autonomous.localization.Localization;

public class AutonCore {
    public static ElapsedTime runtime;

    public void runCore(double initialX, double initialY, double initialTheta, LinearOpMode opMode) {
        runtime = new ElapsedTime();
        Hardware hardware = new Hardware(opMode.hardwareMap);
        Vuforia vuforia = new Vuforia(hardware);
        Localization localization = new Localization(hardware, vuforia, opMode.telemetry, initialX, initialY, initialTheta);
        Instructions instructions = new Instructions(hardware, localization, runtime, opMode.telemetry, opMode, vuforia, initialX, initialY, initialTheta);

        opMode.waitForStart();

        runtime.reset();

        /*do {
            Constants.INIT_THETA = hardware.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS).secondAngle;
        } while (runtime.milliseconds() < 500);*/

        runtime.reset();

        instructions.runTasks();
        opMode.stop();
    }
}
