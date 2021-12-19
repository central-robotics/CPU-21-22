package org.firstinspires.ftc.teamcode.autonomous.waypoint;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.autonomous.Constants;
import org.firstinspires.ftc.teamcode.autonomous.control.PID;
import org.firstinspires.ftc.teamcode.autonomous.hardware.Hardware;
import org.firstinspires.ftc.teamcode.autonomous.localization.Localization;
import org.firstinspires.ftc.teamcode.autonomous.localization.Position;
import org.firstinspires.ftc.teamcode.autonomous.localization.Velocity;

public class Drive {
    private Localization localization;
    private Hardware hardware;
    private PID controller;
    private PID thetaController;
    private Position position;
    private ElapsedTime runtime;
    private LinearOpMode opMode;

    public Drive(Localization localization, Hardware hardware, ElapsedTime runtime, LinearOpMode opMode) {
        this.localization = localization;
        this.hardware = hardware;
        this.runtime = runtime;
        this.opMode = opMode;

        initializeDrive();
    }
    private void initializeDrive()
    {
        PIDCoefficients coefficients = new PIDCoefficients(0.005, 0.00001, 0);
        PIDCoefficients thetaCoefficients = new PIDCoefficients(0.07, 0.0001, 0);

        controller = new PID(coefficients);
        thetaController = new PID(thetaCoefficients);

        position = new Position();
    }

    public void driveToTarget(Position destination)
    {
        boolean thetaFinished = false;
        //Assume that starting position has been reached. Drive to target specified by waypoint.
        while(((Math.abs(destination.x - position.x) > 5) || (Math.abs(destination.y - position.y) > 5) || !thetaFinished) && !opMode.isStopRequested()) {
            thetaFinished = false;

            double thetaError = destination.t - position.t;
            boolean isCounterClockwise = false;

            if ((thetaError) > 0 && (thetaError < Math.PI) ) {
                isCounterClockwise = true;
            }

            if ((thetaError) < 0 && (thetaError < -Math.PI)) {
                isCounterClockwise = true;
                thetaError = destination.t - position.t + (2 * Math.PI);
            }

            if (thetaError < Constants.THETA_TOLERANCE){
                thetaFinished = true;
            }

            setMotorPowers(destination, thetaError, isCounterClockwise);
        }

        controller.resetSum();
    }

    public void setMotorPowers(Position waypointPos, double thetaError, boolean isCounterClockwise)
    {
        if (opMode.isStopRequested())
            return;

        position = localization.getRobotPosition();
        localization.increment(position);

        Velocity velocity = localization.getRobotVelocity(runtime);
        double orientation, magnitude, negOutput, posOutput;

        if (waypointPos.x - position.x > 0)
            orientation = Math.atan(controller.getSlope(waypointPos, position)) - Math.PI / 4 - position.t;
        else
            orientation = Math.atan(controller.getSlope(waypointPos, position)) + Math.PI - Math.PI / 4 - position.t;

        double error = Math.sqrt(Math.pow(waypointPos.y - position.y, 2) + Math.pow(waypointPos.x - position.x, 2));
        double speed = Math.sqrt(Math.pow(velocity.dy, 2) + Math.pow(velocity.dx, 2));

        magnitude = controller.getOutput(error, speed);

        if (error < 3) { // Make magnitude 0 if error is too low to matter
            magnitude = 0;
        }

        negOutput = magnitude * Math.sin(orientation);
        if (orientation == 0)
            posOutput = negOutput;
        else
            posOutput = magnitude * Math.cos(orientation);

        double thetaOutput = thetaController.getOutput(Math.abs(thetaError), 0);

        if (Math.abs(thetaError) < Constants.THETA_TOLERANCE) { // Set thetaOutput to 0 if thetaError is negligible
            thetaOutput = 0;
        }

        hardware.setMotorValuesWithRotation(0.1 * posOutput, 0.1 * negOutput, (isCounterClockwise ? -1 : 1) * thetaOutput);
    }
}
