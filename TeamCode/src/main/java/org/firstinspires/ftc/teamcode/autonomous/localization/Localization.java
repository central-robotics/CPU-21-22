package org.firstinspires.ftc.teamcode.autonomous.localization;

import org.firstinspires.ftc.teamcode.autonomous.hardware.Hardware;

public class Localization {
    private Position robotPosition; //Current robot position. This is used for comparing current robot position to previously recorded robot position.
    private Encoder encoder; //Contains all logic for encoder based localization.
    private Vision vision; //Contains all logic for vision based localization.
    private Hardware _hardware; //Robot hardware for pasing to encoder and vision classes.

    public Localization(Hardware hardware)
    {
        _hardware = hardware;
        encoder = new Encoder(_hardware);
        vision = new Vision(_hardware);
    }


}
