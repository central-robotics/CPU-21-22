package org.firstinspires.ftc.teamcode.autonomous.waypoint;

import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.teamcode.autonomous.PID;
import org.firstinspires.ftc.teamcode.autonomous.hardware.Hardware;
import org.firstinspires.ftc.teamcode.autonomous.localization.Localization;

import java.util.ArrayList;

public class Navigation {
    private Hardware _hardware;
    private Localization _localization;

    private PID xController;
    private PID yController;
    private PID tController;

    /*
    Holds waypoints that we can drive to. This allows for the robot to split a move up into
    multiple linear movements. This allows the robot to avoid obstacles and for movement to be planned.

    In the future, this feature may be used to add a GUI for motion planning.
     */
    private ArrayList<Waypoint> waypoints;
    private int index = 0;

    public Navigation(Hardware hardware, Localization localization)
    {
        _hardware = hardware;
        _localization = localization;

        PIDCoefficients coefficients = new PIDCoefficients(0, 0, 0);

        xController = new PID(coefficients);
        yController = new PID(coefficients);
        tController = new PID(coefficients);

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
    }

    private void driveToTarget(Waypoint waypoint)
    {
        //Assume that starting position has been reached. Drive to target specified by waypoint.
    }
}
