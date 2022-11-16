package org.firstinspires.ftc.teamcode.autonomous.hardware;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.autonomous.Constants;
//import org.openftc.easyopencv.OpenCvCamera;
//import org.openftc.easyopencv.OpenCvCameraFactory;

import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicLong;

public class Hardware {
    public DcMotor leftFrontMotor, rightFrontMotor, leftBackMotor, rightBackMotor;
    public DcMotor carouselMotor;
    public DcMotor armMotor;
    public DcMotorEx intakeMotor;
    public Servo boxServo, constraintServo, sweeperServo;
    public BNO055IMU gyro;
    public WebcamName camera;
//    public OpenCvCamera openCvCamera;
    public HardwareMap map;
    public boolean intakeSpinning = false;
    public AtomicInteger boxServoPosition = new AtomicInteger(700);

    public Hardware (HardwareMap hardware)
    {
        map = hardware;
//        initializeGyro(hardware);
        initializeDriveMotors(hardware);
//        initializeAccessoryMotors(hardware);
//        initializeWebcam(hardware);
    }

    private void initializeDriveMotors(HardwareMap hardware)
    {
        leftFrontMotor = hardware.dcMotor.get("leftFrontMotor");
        leftFrontMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftBackMotor = hardware.dcMotor.get("leftBackMotor");
        leftBackMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightFrontMotor = hardware.dcMotor.get("rightFrontMotor");
        rightFrontMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightBackMotor = hardware.dcMotor.get("rightBackMotor");
        rightBackMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setMotorValues(double posinput, double neginput)
    {
        leftFrontMotor.setPower(-posinput);
        rightFrontMotor.setPower(neginput);
        leftBackMotor.setPower(-neginput);
        rightBackMotor.setPower(posinput);
    }

    public void setMotorValuesWithRotation(double posinput, double neginput, double rotinput)
    {
        leftFrontMotor.setPower(-posinput-rotinput);
        rightFrontMotor.setPower(neginput-rotinput);
        leftBackMotor.setPower(-neginput-rotinput);
        rightBackMotor.setPower(posinput-rotinput);
    }

    public void setAllMotorPowers(double input)
    {
        leftFrontMotor.setPower(input);
        rightFrontMotor.setPower(input);
        leftBackMotor.setPower(input);
        rightBackMotor.setPower(input);
    }

    private void initializeAccessoryMotors(HardwareMap hardware)
    {
        carouselMotor = hardware.dcMotor.get("carouselMotor");
        carouselMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        carouselMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armMotor = hardware.dcMotor.get("slideMotor");
        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeMotor = hardware.get(DcMotorEx.class, "intakeMotor");
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        boxServo = hardware.get(Servo.class, "boxServo");
        constraintServo = hardware.get(Servo.class, "constraintServo");
        sweeperServo = hardware.get(Servo.class, "sweeperServo");
    }

    private void initializeGyro(HardwareMap hardware)
    {
        BNO055IMU.Parameters params = new BNO055IMU.Parameters();
        params.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        params.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        gyro = hardware.get(BNO055IMU.class, "imu");
        gyro.initialize(params);
    }

    private void initializeWebcam(HardwareMap hardware)
    {
//        camera = hardware.get(WebcamName.class, "cam");
//        openCvCamera = OpenCvCameraFactory.getInstance().createWebcam(camera);
    }
}
