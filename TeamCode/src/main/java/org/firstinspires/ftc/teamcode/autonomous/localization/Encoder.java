package org.firstinspires.ftc.teamcode.autonomous.localization;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class Encoder
{
    public void initializeEncoders()
    {

    }

    class Location {
        VectorF translation;
        Orientation orientation;
    }
}