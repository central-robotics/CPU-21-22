package org.firstinspires.ftc.teamcode.autonomous.waypoint;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.autonomous.Constants;
import org.firstinspires.ftc.teamcode.autonomous.actions.Actions;
import org.firstinspires.ftc.teamcode.autonomous.hardware.Hardware;
import org.firstinspires.ftc.teamcode.autonomous.localization.Localization;
import org.firstinspires.ftc.teamcode.autonomous.localization.Position;
import org.firstinspires.ftc.teamcode.autonomous.waypoint.path.Path;

import java.util.ArrayList;

public class Navigation {
    public final Drive drive;
    private final Hardware hardware;
    private final Actions actions;
    private final ElapsedTime runtime;

    private final Telemetry telem;
    private final ArrayList<Waypoint> waypoints;
    private final ArrayList<Path> pipeline;
    private final LinearOpMode opMode;
    public Navigation(Hardware hardware, Localization localization, ElapsedTime runtime, Actions actions, Telemetry telemetry, LinearOpMode opMode)
    {
        this.opMode = opMode;
        this.runtime = runtime;
        this.drive = new Drive(localization, hardware, runtime, opMode);
        this.actions = actions;
        this.hardware = hardware;

        telem = telemetry;

        waypoints = new ArrayList<>();
        pipeline = new ArrayList<>();
    }

    public void addPathToPipeline(Path path)
    {
        if (!Constants.IS_BLUE_TEAM)
            for (Position pose : path.points)
            {
                pose.x *= -1;
            }

        pipeline.add(path);
    }

    public void addWayPointToQueue(Waypoint waypoint)
    {
        if (!Constants.IS_BLUE_TEAM) {
            waypoint.startingPos.x *= -1;
            waypoint.targetPos.x *= -1;
        }
        waypoints.add(waypoint);
    }

    public void executeTask()
    {
        for (int i = 0; i < waypoints.size(); i++)
        {
            Waypoint waypoint = waypoints.get(i);

            if (opMode.isStopRequested())
                break;

            double time = runtime.milliseconds();
            hardware.setAllMotorPowers(0);

            while (runtime.milliseconds() - time < 500)
            {
                //nothing
            }

            telem.addData("starting T", waypoint.startingPos.t);
            telem.addData("target T", waypoint.targetPos.t);

            drive.driveToTarget(waypoint.startingPos);

            if (opMode.isStopRequested())
                break;

            drive.driveToTarget(waypoint.targetPos);

            if (opMode.isStopRequested())
                break;

            actions.executeTask(i);
        }

        waypoints.clear();
    }
}
