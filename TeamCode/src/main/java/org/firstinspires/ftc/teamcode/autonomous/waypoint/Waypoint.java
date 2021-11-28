package org.firstinspires.ftc.teamcode.autonomous.waypoint;

import org.firstinspires.ftc.teamcode.autonomous.localization.Position;

public class Waypoint {
    public Position targetPos;
    public Position startingPos;

    public Waypoint(Position startingPosition, Position targetPosition)
    {
        startingPos = startingPosition;
        targetPos = targetPosition ;
    }
}
