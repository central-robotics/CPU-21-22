package org.firstinspires.ftc.teamcode.autonomous.hardware;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.autonomous.AutonCore;

public final class Webcam extends AutonCore {
    public static WebcamName webcam;

    public void initializeWebcam()
    {
        webcam = hardwareMap.get(WebcamName.class, "PrimaryWebcam");
    }
}
