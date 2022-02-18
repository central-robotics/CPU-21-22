package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class TeleOpHardware {
    public DcMotor lf, lb, rf, rb;
    public DcMotor carouselMotor, slideMotor, intakeMotor;
    public Servo boxServo;
    public Servo constraintServo;
    public Servo sweeperServo;
    public BNO055IMU imu;

    public void init(HardwareMap hardwareMap)
    {
        lf = hardwareMap.dcMotor.get("leftFrontMotor");
        lf.setDirection(DcMotorSimple.Direction.FORWARD);

        rf = hardwareMap.dcMotor.get("rightFrontMotor");
        rf.setDirection(DcMotorSimple.Direction.FORWARD);

        rb = hardwareMap.dcMotor.get("rightBackMotor");
        rb.setDirection(DcMotorSimple.Direction.FORWARD);

        lb = hardwareMap.dcMotor.get("leftBackMotor");
        lb.setDirection(DcMotorSimple.Direction.FORWARD);

        carouselMotor = hardwareMap.dcMotor.get("carouselMotor");
        carouselMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        slideMotor = hardwareMap.dcMotor.get("slideMotor");
        slideMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intakeMotor = hardwareMap.dcMotor.get("intakeMotor");
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        boxServo = hardwareMap.get(Servo.class, "boxServo");

        sweeperServo = hardwareMap.get(Servo.class, "sweeperServo");

        constraintServo = hardwareMap.get(Servo.class, "constraintServo");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.loggingEnabled = false;

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }

}
