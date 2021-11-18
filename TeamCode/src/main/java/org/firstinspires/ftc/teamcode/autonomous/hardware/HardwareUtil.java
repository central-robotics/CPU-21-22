package org.firstinspires.ftc.teamcode.autonomous.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;

public class HardwareUtil {
    public static Motors motors;
    public static Gyro gyro;
    public static Webcam webcam;

    public void initializeRobot(HardwareMap hardwareMap)
    {
        motors = new Motors();
        motors.initializeMotors(hardwareMap);
        gyro = new Gyro();
        gyro.initializeIMU(hardwareMap);
        webcam = new Webcam();
        webcam.initializeWebcam(hardwareMap);
    }
}
