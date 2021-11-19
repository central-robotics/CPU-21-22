package org.firstinspires.ftc.teamcode.autonomous.localization;

import org.firstinspires.ftc.teamcode.autonomous.hardware.Hardware;

public class Localization {
    private Position robotPosition; //Current robot position. This is used for comparing current robot position to previously recorded robot position.
    private Encoder encoder; //Contains all logic for encoder based localization.
    private Vision vision; //Contains all logic for vision based localization.
    private Hardware _hardware; //Robot hardware for passing to encoder and vision classes.

    public Localization(Hardware hardware)
    {
        _hardware = hardware;
        robotPosition = new Position();
        robotPosition.y = 0;
        robotPosition.x = 0;
        robotPosition.t = 0;
        encoder = new Encoder(_hardware);
        vision = new Vision(_hardware);
    }

    public Position getRobotPosition()
    {
        Position visionRobotPosition = vision.getRobotPosition();

        if (visionRobotPosition != null)
        {
            robotPosition = visionRobotPosition; //This will allow encoder localization to correct to these new values.
            return visionRobotPosition;
        }

        return encoder.getRobotPosition(robotPosition); //If we can't see vision targets, return encoder based location.
    }


}
