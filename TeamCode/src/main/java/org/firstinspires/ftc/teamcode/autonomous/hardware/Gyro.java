package org.firstinspires.ftc.teamcode.autonomous.hardware;

import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.teamcode.autonomous.AutonCore;

public final class Gyro extends AutonCore {
    public static BNO055IMU imu;

    public void initializeIMU()
    {
        BNO055IMU.Parameters params = new BNO055IMU.Parameters();
        params.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        params.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(params);
    }
}
