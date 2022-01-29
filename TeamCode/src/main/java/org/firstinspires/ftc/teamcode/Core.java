package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public class Core extends OpMode {
    DcMotor leftfront, rightfront, leftback, rightback, armMotor, carousel, intake;
    Servo boxServo;
    BNO055IMU imu;
    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

    public void loop(){}

    public void init()
    {
        leftfront = hardwareMap.dcMotor.get("leftFrontMotor");
        leftfront.setDirection(DcMotorSimple.Direction.FORWARD);

        rightfront = hardwareMap.dcMotor.get("rightFrontMotor");
        rightfront.setDirection(DcMotorSimple.Direction.REVERSE);

        leftback = hardwareMap.dcMotor.get("leftBackMotor");
        leftback.setDirection(DcMotorSimple.Direction.FORWARD);

        rightback = hardwareMap.dcMotor.get("rightBackMotor");
        rightback.setDirection(DcMotorSimple.Direction.FORWARD);

        armMotor = hardwareMap.dcMotor.get("slideMotor");
        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        carousel = hardwareMap.dcMotor.get("carouselMotor");
        carousel.setDirection(DcMotorSimple.Direction.FORWARD);

        intake = hardwareMap.dcMotor.get("intakeMotor");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);



        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.loggingEnabled = false;

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }

    public void move(double posinput, double neginput, double rotinput)
    {
        leftfront.setPower(0.5* (posinput+rotinput));
        rightfront.setPower(0.5 * (-neginput+rotinput));
        leftback.setPower(0.5 * (neginput+rotinput));
        rightback.setPower(0.5 * (-posinput+rotinput));
    }

    public void moveCarousel(double carouselpower)
    {
            carousel.setPower(carouselpower);
    }

    public void moveIntake(double power) {
        intake.setPower(power);
    }

    public void moveSlider(double power)
    {
        armMotor.setPower(power);
    }
}
