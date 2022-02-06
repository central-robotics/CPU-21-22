package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.autonomous.actions.util.ObjectDetector;
import org.firstinspires.ftc.teamcode.autonomous.instructions.Instructions;
import org.firstinspires.ftc.teamcode.autonomous.vision.Vuforia;
import org.firstinspires.ftc.teamcode.autonomous.hardware.Hardware;
import org.firstinspires.ftc.teamcode.autonomous.localization.Localization;

public class AutonCore {
    public static ElapsedTime runtime;
    public static Telemetry telem;

    public void runCore(double initialX, double initialY, double initialTheta, LinearOpMode opMode) {
        runtime = new ElapsedTime();
        Hardware hardware = new Hardware(opMode.hardwareMap);
        Vuforia vuforia = null;
        Constants.CURRENT_INITIAL_THETA = initialTheta;
        ObjectDetector detector = new ObjectDetector(hardware, vuforia);
        Localization localization = new Localization(hardware, vuforia, opMode.telemetry, initialX, initialY, initialTheta);
        Instructions instructions = new Instructions(hardware, localization, runtime, opMode.telemetry, opMode, vuforia, detector, initialX, initialY, initialTheta);
        telem = opMode.telemetry;
        opMode.waitForStart();

        hardware.constraintServo.setPosition(0);

        runtime.reset();

        /*do {
            Constants.INIT_THETA = hardware.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS).secondAngle;
        } while (runtime.milliseconds() < 500);*/

        runtime.reset();

        /*while(true)
        {
            Position pos = localization.getRobotPosition();
            localization.increment(pos);
            opMode.telemetry.addData("X", pos.x);
            opMode.telemetry.addData("Y", pos.y);
            opMode.telemetry.addData("T", pos.t);
            opMode.telemetry.addData("0", hardware.leftFrontMotor.getCurrentPosition());
            opMode.telemetry.addData("1", hardware.rightFrontMotor.getCurrentPosition());
            opMode.telemetry.addData("2", hardware.rightBackMotor.getCurrentPosition());
            opMode.telemetry.addData("3", hardware.leftBackMotor.getCurrentPosition());
            opMode.telemetry.update();
        }*/

        instructions.runTasks();
        opMode.stop();
    }
}
