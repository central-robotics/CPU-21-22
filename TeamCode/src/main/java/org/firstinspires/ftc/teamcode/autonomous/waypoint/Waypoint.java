package org.firstinspires.ftc.teamcode.autonomous.waypoint;

import org.firstinspires.ftc.teamcode.autonomous.localization.Position;

public class Waypoint {
    private Position targetPos;
    private Position startingPos;

    public Waypoint(Position startingPosition, Position targetPosition)
    {
        startingPos = startingPosition;
        targetPos = targetPosition ;
    }

    public double resolveDist(Position currentPosition, Position targetPosition)
    {
        return Math.hypot(targetPosition.x - currentPosition.x, targetPosition.y - currentPosition.y);
    }
}