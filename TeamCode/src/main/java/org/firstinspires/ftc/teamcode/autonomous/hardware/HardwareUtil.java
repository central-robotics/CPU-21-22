package org.firstinspires.ftc.teamcode.autonomous.hardware;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;

public final class HardwareUtil {
    public static Motors motors;
    public static Gyro gyro;
    public static Webcam webcam;

    public void InitializeRobot()
    {
        motors = new Motors();
        motors.InitializeMotors();
        gyro = new Gyro();
        gyro.initializeIMU();
        webcam = new Webcam();
        webcam.initializeWebcam();
    }
}
