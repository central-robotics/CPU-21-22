package org.firstinspires.ftc.teamcode.autonomous.waypoint;

import org.firstinspires.ftc.teamcode.autonomous.localization.Localizer;
import org.firstinspires.ftc.teamcode.autonomous.localization.Position;

public class Waypoint {
    private Position _targetLocation;
    private Position _startingPosition;
    private int _instruction;

    public Waypoint(Position currentPosition, Position startingPosition, Position targetLocation, int instruction)
    {
        _startingPosition = startingPosition;
        _targetLocation = targetLocation;
        _instruction = instruction;

        float percentErrorX = ((currentPosition.x - startingPosition.x) / (startingPosition.x) ) * 100;
        float percentErrorY = ((currentPosition.y - startingPosition.y) / (startingPosition.y) ) * 100;

        if (percentErrorX > 10 || percentErrorY > 10) //we need to move the robot to the starting location as we are too far from the starting point to be safe
        {
            driveToStart();
        }
    }

    private void driveToStart()
    {

    }
}
