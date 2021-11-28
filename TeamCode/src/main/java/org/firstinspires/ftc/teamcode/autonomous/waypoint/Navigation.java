package org.firstinspires.ftc.teamcode.autonomous.waypoint;

import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.autonomous.control.Controller;
import org.firstinspires.ftc.teamcode.autonomous.control.RelativeRobotPos;
import org.firstinspires.ftc.teamcode.autonomous.hardware.Hardware;
import org.firstinspires.ftc.teamcode.autonomous.localization.Localization;
import org.firstinspires.ftc.teamcode.autonomous.localization.Position;
import org.firstinspires.ftc.teamcode.autonomous.localization.Velocity;

import java.util.ArrayList;

public class Navigation {
    private Hardware _hardware;
    private Localization _localization;
    private ElapsedTime runtime;
    private Controller controller;
    private Position position = new Position();
    private Velocity velocity = new Velocity();
    private RelativeRobotPos output = new RelativeRobotPos();

    /*
    Holds waypoints that we can drive to. This allows for the robot to split a move up into
    multiple linear movements. This allows the robot to avoid obstacles and for movement to be planned.

    In the future, this feature may be used to add a GUI for motion planning.
     */
    private ArrayList<Waypoint> waypoints;
    private int index = 0;

    public Navigation(Hardware hardware, Localization localization, ElapsedTime runtime)
    {
        this.runtime = runtime;
        _hardware = hardware;
        _localization = localization;

        PIDCoefficients coefficients = new PIDCoefficients(0.25, 0, 0);

        controller = new Controller(coefficients);

        waypoints = new ArrayList<>();
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
            driveToStart(waypoint);
            driveToTarget(waypoint);
        }

        waypoints.clear();
        index = 0;
    }

    private void driveToStart(Waypoint waypoint)
    {
        //Drive to starting location of waypoint. Robot will take the shortest possible path.
        while((Math.abs(waypoint.startingPos.x - position.x) > 5) ||
                (Math.abs(waypoint.startingPos.y - position.y) > 5))
        {
            position = _localization.getRobotPosition();
            _localization.increment(position);
            velocity = _localization.getRobotVelocity(runtime);

            double orientation = (position.x > 0) ? (Math.atan(-position.y / position.x) - Math.PI / 4):
                    (Math.atan(-position.y / position.x) + Math.PI - Math.PI / 4);

            double targetOrientation = (waypoint.startingPos.x > 0) ? (Math.atan(-waypoint.startingPos.y / waypoint.startingPos.x) - Math.PI / 4):
                    (Math.atan(-waypoint.startingPos.y / waypoint.startingPos.x) + Math.PI - Math.PI / 4);

            RelativeRobotPos translated_pos = new RelativeRobotPos();
            RelativeRobotPos translated_target = new RelativeRobotPos();

            // Pass that angle through a pair of wave functions to get the power for each corresponding pair of parallel wheels
            translated_pos.neg = 0.45 * Math.sin(orientation);
            translated_pos.pos = (orientation != 0) ? 0.45 * Math.cos(orientation) :
                    translated_pos.neg;

            translated_target.neg = 0.45 * Math.sin(targetOrientation);
            translated_target.pos = (targetOrientation != 0) ? 0.45 * Math.cos(targetOrientation) :
                    translated_target.neg;

            output = controller.eval(translated_pos, translated_target, velocity);

            _hardware.setMotorValues(output.pos, output.neg);
        }
    }

    private void driveToTarget(@org.jetbrains.annotations.NotNull Waypoint waypoint)
    {
        //Assume that starting position has been reached. Drive to target specified by waypoint.
        while((Math.abs(waypoint.targetPos.x - position.x) > 5) ||
                (Math.abs(waypoint.targetPos.y - position.y) > 5))
        {
            position = _localization.getRobotPosition();
            _localization.increment(position);
            velocity = _localization.getRobotVelocity(runtime);

            output = controller.eval(position, waypoint.targetPos, velocity);

            _hardware.setMotorValues(output.pos, output.neg);
        }
    }
}
