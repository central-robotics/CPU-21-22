package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class DriveLoop {
    double positive_power, negative_power, rot_power;
    double joystick_x, joystick_y, joystick_power;
    double orientation;
    double intakePower;
    boolean turboEnabled;
    Orientation gyro_angles;
    long prevTime = System.currentTimeMillis();

    public void loop(TeleOpHardware hardware, OpMode opMode)
    {
        if (opMode.gamepad1.right_trigger != 0)
        {
            turboEnabled = true;
        } else {
            turboEnabled = false;
        }
        double sliderPos = hardware.slideMotor.getCurrentPosition();
        boolean changeBoxPos = false;

        if (Math.abs(sliderPos) > 300)
            changeBoxPos = true;

        if (opMode.gamepad1.a)
        {
            moveIntake(1, hardware);
        }

        if (opMode.gamepad1.b)
        {
            moveCarousel(0.5, hardware);
        } else if (!opMode.gamepad1.b)
        {
            moveCarousel(0.0001, hardware);
        }

        if (opMode.gamepad1.y)
        {
            moveBoxServo(.99, hardware);
        } else
        {
            if (changeBoxPos)
                moveBoxServo(0.5, hardware);
            else
                moveBoxServo(0.7, hardware);

            changeBoxPos = false;
        }

        if (opMode.gamepad1.x)
            moveSweeper(0.49, hardware);
        else
            moveSweeper(1, hardware);


        if (opMode.gamepad1.right_bumper)
        {
            moveSlider(.75, hardware);
        } else if (opMode.gamepad1.left_bumper)
            moveSlider(-.1, hardware);

        if (!opMode.gamepad1.right_bumper && !opMode.gamepad1.left_bumper)
            moveSlider(0.00001, hardware);

        prevTime = System.currentTimeMillis();
        // Get all the info we from the gamepad
        joystick_y = opMode.gamepad1.left_stick_y > 0 ? Math.pow(opMode.gamepad1.left_stick_y, 2) :
                -Math.pow(opMode.gamepad1.left_stick_y, 2);
        joystick_x = (opMode.gamepad1.left_stick_x == 0) ? 0.000001 :
                (opMode.gamepad1.left_stick_x > 0 ? Math.pow(opMode.gamepad1.left_stick_x, 2) :
                        -Math.pow(opMode.gamepad1.left_stick_x, 2));

        rot_power = 0.7 * (opMode.gamepad1.right_stick_x);



        // Find out the distance of the joystick from resting position to control speed
        joystick_power = Math.sqrt(Math.pow(joystick_x, 2) + Math.pow(joystick_y, 2));

        // Pull raw orientation values from the gyro
        gyro_angles = hardware.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        double theta = gyro_angles.firstAngle; // Add pi for CPU's robot


        // Turn the joystick coordinates into an angle in radians
        orientation = (joystick_x > 0) ? (Math.atan(-joystick_y / joystick_x) - Math.PI / 4)  - theta :
                (Math.atan(-joystick_y/joystick_x) + Math.PI - Math.PI / 4) - theta ;


        // Pass that angle through a pair of wave functions to get the power for each corresponding pair of parallel wheels
        negative_power = (joystick_power * Math.sin(orientation));
        positive_power = (orientation != 0) ? (joystick_power * Math.cos(orientation)) :
                negative_power;

        if (turboEnabled){
            move(2 * positive_power, 2 * negative_power, 2 * rot_power, hardware);
        }
        else {
            move(positive_power, negative_power, rot_power, hardware);
        }
    }

    public void moveSlider(double power, TeleOpHardware hardware)
    {
        hardware.slideMotor.setPower(power);
    }

    public void moveIntake(double power, TeleOpHardware hardware)
    {
        if (intakePower == 1) {
            hardware.intakeMotor.setPower(0.0001);
            intakePower = 0.0001;
        } else {
            hardware.intakeMotor.setPower(1);
            intakePower = 1;
        }
    }

    public void moveCarousel(double power, TeleOpHardware hardware)
    {
        if (TeleOpConstants.isBlueOpMode)
        {
            hardware.carouselMotor.setPower(-power);
        } else
        {
            hardware.carouselMotor.setPower(power);
        }
    }

    public void moveBoxServo(double pos, TeleOpHardware hardware)
    {
        hardware.boxServo.setPosition(pos);
    }

    public void moveSweeper(double pos, TeleOpHardware hardware)
    {
        hardware.sweeperServo.setPosition(pos);
    }

    public void move(double posinput, double neginput, double rotinput, TeleOpHardware hardware)
    {
        hardware.lf.setPower(.6* (-posinput-rotinput));
        hardware.rf.setPower(.6 * (neginput-rotinput));
        hardware.lb.setPower(.6 * (-neginput-rotinput));
        hardware.rb.setPower(.6 * (posinput-rotinput));
    }
}
