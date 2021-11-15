package org.firstinspires.ftc.teamcode.autonomous.waypoint;

import org.firstinspires.ftc.teamcode.autonomous.localization.Localizer;
import org.firstinspires.ftc.teamcode.autonomous.localization.Position;

public class Waypoint {
    public Position targetPos;
    public Position startingPos;
    private int _instruction;

    public Waypoint(Position startingPosition, Position targetPosition, int instruction)
    {
        startingPos = startingPosition;
        targetPos = targetPosition ;
        _instruction = instruction;
    }

    public float resolveDist(Position currentPosition, Position targetPosition)
    {
        return (float) Math.hypot(targetPosition.x - currentPosition.x, targetPosition.y - currentPosition.y);
    }
}
