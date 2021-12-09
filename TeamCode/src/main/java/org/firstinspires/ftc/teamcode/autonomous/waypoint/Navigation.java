package org.firstinspires.ftc.teamcode.autonomous.waypoint;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.autonomous.AutonCore;
import org.firstinspires.ftc.teamcode.autonomous.actions.Actions;
import org.firstinspires.ftc.teamcode.autonomous.control.PID;
import org.firstinspires.ftc.teamcode.autonomous.hardware.Hardware;
import org.firstinspires.ftc.teamcode.autonomous.localization.Localization;
import org.firstinspires.ftc.teamcode.autonomous.localization.Position;
import org.firstinspires.ftc.teamcode.autonomous.localization.Velocity;

import java.util.ArrayList;

public class Navigation {
    private final Hardware _hardware;
    private final Localization _localization;
    private final Actions _actions;
    private final ElapsedTime runtime;
    private final PID controller;
    private final PID thetaController;
    private final Telemetry telem;
    private Position position = new Position();
    /*
    Holds waypoints that we can drive to. This allows for the robot to split a move up into
    multiple linear movements. This allows the robot to avoid obstacles and for movement to be planned.

    In the future, this feature may be used to add a GUI for motion planning.
     */
    private ArrayList<Waypoint> waypoints;
    private int index = 0;
    private final LinearOpMode opMode;

    public Navigation(Hardware hardware, Localization localization, ElapsedTime runtime, Actions actions, Telemetry telemetry, LinearOpMode opMode)
    {
        telem = telemetry;
        this.runtime = runtime;
        _actions = actions;
        _hardware = hardware;
        _localization = localization;

        PIDCoefficients coefficients = new PIDCoefficients(0.002, 0, 0);
        PIDCoefficients thetaCoefficients = new PIDCoefficients(0.05, 0, 0);

        controller = new PID(coefficients);
        thetaController = new PID(thetaCoefficients);

        waypoints = new ArrayList<>();
        this.opMode = opMode;
    }

    public void addWayPointToQueue(Waypoint waypoint)
    {
        waypoints.add(index, waypoint);
        index++;
    }

    public void executeTask()
    {
        for (Waypoint waypoint : waypoints)
        {
            if (opMode.isStopRequested())
                break;

            driveToStart(waypoint);

            if (opMode.isStopRequested())
                break;

            driveToTarget(waypoint);

            if (opMode.isStopRequested())
                break;

            _actions.executeTask(index);
            index++;
        }

        waypoints.clear();
    }

    private void driveToStart(Waypoint waypoint)
    {
        //Drive to starting location of waypoint. Robot will take the shortest possible path.
        while((Math.abs(waypoint.startingPos.x - position.x) > 5) ||
                (Math.abs(waypoint.startingPos.y - position.y) > 5) && !opMode.isStopRequested())
        {
           moveToTarget(waypoint.startingPos);
        }
    }

    private void driveToTarget(Waypoint waypoint)
    {
        //Assume that starting position has been reached. Drive to target specified by waypoint.
        while(((Math.abs(waypoint.targetPos.x - position.x) > 5) || (Math.abs(waypoint.targetPos.y - position.y) > 5)) && !opMode.isStopRequested()) {
            moveToTarget(waypoint.targetPos);
        }
    }

    private void moveToTarget(Position waypointPos)
    {
        if (opMode.isStopRequested())
            return;

        position = _localization.getRobotPosition(telem);
        _localization.increment(position);
        Velocity velocity = _localization.getRobotVelocity(runtime);
        double orientation, magnitude, negOutput, posOutput;

        if (waypointPos.x - position.x > 0)
            orientation = Math.atan(controller.getSlope(waypointPos, position)) - Math.PI / 4;
        else
            orientation = Math.atan(controller.getSlope(waypointPos, position)) + Math.PI - Math.PI / 4;

        double error = Math.sqrt(Math.pow(waypointPos.y - position.y, 2) + Math.pow(waypointPos.x - position.x, 2));
        double speed = Math.sqrt(Math.pow(velocity.dy, 2) + Math.pow(velocity.dx, 2));

        magnitude = controller.getOutput(error, speed);

        negOutput = magnitude * Math.sin(orientation);
        if (orientation == 0)
            posOutput = negOutput;
        else
            posOutput = magnitude * Math.cos(orientation);

        telem.addData("X", position.x);
        telem.addData("Y", position.y);
        telem.addData("T", position.t);
        telem.addData("Velocity", Math.sqrt(Math.pow(velocity.dx, 2) + Math.pow(velocity.dy, 2)));
        telem.update();

        _hardware.setMotorValues(0.1 * posOutput, 0.1 * negOutput);
    }
}
