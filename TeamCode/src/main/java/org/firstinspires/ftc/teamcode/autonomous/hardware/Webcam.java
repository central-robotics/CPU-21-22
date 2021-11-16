package org.firstinspires.ftc.teamcode.autonomous.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.autonomous.AutonCore;

public class Webcam {
    public static WebcamName webcam;

    public void initializeWebcam(HardwareMap hardwareMap)
    {
        webcam = hardwareMap.get(WebcamName.class, "PrimaryWebcam");
    }
}
