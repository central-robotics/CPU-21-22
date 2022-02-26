package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.teleop.accessory.Carousel;
import org.firstinspires.ftc.teamcode.teleop.accessory.Intake;
import org.firstinspires.ftc.teamcode.teleop.nav.Drive;

public class DriveLoop {
    boolean turboEnabled;
    long prevTime = System.currentTimeMillis();
    double carouselCooldown = 0, intakeCooldown = 0;

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

        if (Math.abs(sliderPos) > 100)
            changeBoxPos = true;

        if (opMode.gamepad1.a) {
            if (intakeCooldown == 0)
            {
                Intake.spinIntake(hardware, opMode.gamepad1);
                intakeCooldown = System.currentTimeMillis() + 200;
            } else
                if (prevTime > intakeCooldown)
                    intakeCooldown = 0;
        }

        if (opMode.gamepad1.b) {
            if (carouselCooldown == 0)
            {
                Carousel.spinCarousel(hardware);
                carouselCooldown = System.currentTimeMillis() + 200;
            } else
            if (prevTime > carouselCooldown)
                carouselCooldown= 0;
        }

        if (opMode.gamepad1.y)
        {
            moveBoxServo(0, hardware);
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
            moveSlider(.76, hardware);
        } else if (opMode.gamepad1.left_bumper)
            moveSlider(-.3, hardware);

        if (!opMode.gamepad1.right_bumper && !opMode.gamepad1.left_bumper)
            moveSlider(0.00001, hardware);

        prevTime = System.currentTimeMillis();

        Drive.moveRobot(opMode.gamepad1, hardware, turboEnabled);
    }

    public void moveSlider(double power, TeleOpHardware hardware)
    {
        hardware.slideMotor.setPower(power);
    }


    public void moveBoxServo(double pos, TeleOpHardware hardware)
    {
        hardware.boxServo.setPosition(pos);
    }

    public void moveSweeper(double pos, TeleOpHardware hardware)
    {
        hardware.sweeperServo.setPosition(pos);
    }

}
