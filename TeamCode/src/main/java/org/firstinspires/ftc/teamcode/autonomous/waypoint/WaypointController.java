package org.firstinspires.ftc.teamcode.autonomous.waypoint;

import static org.firstinspires.ftc.teamcode.autonomous.AutonCore.localizer;

public final class WaypointController {
    public static void driveToTarget(Waypoint waypoint)
    {
        driveToStart(waypoint); //we first need to drive to the starting position for the waypoint.
    }

    private static void driveToStart(Waypoint waypoint)
    {
        float percentErrorX = Math.abs(((localizer.robotPos.x - waypoint.startingPos.x) / (waypoint.startingPos.x) ) * 100);
        float percentErrorY = Math.abs(((localizer.robotPos.y - waypoint.startingPos.y) / (waypoint.startingPos.y) ) * 100);

        if (percentErrorX > 10 || percentErrorY > 10) //As long as error is within 10%, we're not going to bother moving to start location.
        {
            float dist =  waypoint.resolveDist(localizer.robotPos, waypoint.startingPos);
            float vectorX =  (float)(0.4 * (waypoint.startingPos.x - localizer.robotPos.x) / dist);
            float vectorY = (float)(0.4 * (waypoint.startingPos.y - localizer.robotPos.y) / dist);
        } else
            return;


    }
}
