package org.firstinspires.ftc.teamcode.autonomous.localization;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.autonomous.hardware.Hardware;

public class Localization {
    private Position newPosition; //Current robot position. This is used for comparing current robot position to previously recorded robot position.
    private Position previousRobotPosition;
    private Encoder encoder; //Contains all logic for encoder based localization.
    private Vision vision; //Contains all logic for vision based localization.
    private Hardware _hardware; //Robot hardware for passing to encoder and vision classes.
    private ElapsedTime runtime;
    private double currentTime;
    private double previousTime;

    public Localization(Hardware hardware, Telemetry telemetry)
    {
        _hardware = hardware;
        newPosition = new Position();
        newPosition.y = 0;
        newPosition.x = 0;
        newPosition.t = 0;
        previousRobotPosition = new Position();
        previousRobotPosition.y = 0;
        previousRobotPosition.x = 0;
        previousRobotPosition.t = 0;
        encoder = new Encoder(_hardware);
        vision = null;
        runtime = new ElapsedTime();
    }

    public void increment(Position _newPosition)
    {
        previousRobotPosition = newPosition;
        newPosition = _newPosition;
        previousTime = currentTime;
    }

    public Position getRobotPosition()
    {
        Position visionRobotPosition = null; //vision.getRobotPosition();

        if (visionRobotPosition != null)
        {
            newPosition = visionRobotPosition; //This will allow encoder localization to correct to these new values.
            return visionRobotPosition;
        }

        return encoder.getRobotPosition(newPosition); //If we can't see vision targets, return encoder based location.
    }

    public Velocity getRobotVelocity(ElapsedTime runtime)
    {
        currentTime = runtime.milliseconds();
        return encoder.getRobotVelocity(previousRobotPosition, newPosition, previousTime, currentTime);
    }
}