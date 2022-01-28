package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.autonomous.control.PID;

@TeleOp
public class Drive extends Core {
    double positive_power, negative_power, rot_power;
    double joystick_x, joystick_y, joystick_power;
    double orientation;
    Orientation gyro_angles;
    long prevTime = System.currentTimeMillis();

    public void loop() {
        if (gamepad1.a)
        {
            moveIntake(1);
        }

        if (gamepad1.square)
            moveIntake(0.0001);

        if (gamepad1.b)
        {
            moveCarousel(0.7);
        } else if (!gamepad1.b)
        {
            moveCarousel(0.0001);
        }

        if (gamepad1.right_bumper)
        {
            moveSlider(.9);
        } else if (gamepad1.left_bumper)
            moveSlider(-.4);

        if (!gamepad1.right_bumper && !gamepad1.left_bumper)
            moveSlider(0.00001);



        prevTime = System.currentTimeMillis();
        // Get all the info we from the gamepad
        joystick_y = gamepad1.left_stick_y;
        joystick_x = (gamepad1.left_stick_x == 0) ? 0.000001 :
                gamepad1.left_stick_x;
        rot_power = 1 * (gamepad1.right_stick_x);



        // Find out the distance of the joystick from resting position to control speed
        joystick_power = Math.sqrt(Math.pow(joystick_x, 2) + Math.pow(joystick_y, 2));

        // Pull raw orientation values from the gyro
        gyro_angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS);
        double theta = gyro_angles.firstAngle; // Add pi for CPU's robot


        // Turn the joystick coordinates into an angle in radians
        orientation = (joystick_x > 0) ? (Math.atan(-joystick_y / joystick_x) - Math.PI / 4)  - theta :
                (Math.atan(-joystick_y/joystick_x) + Math.PI - Math.PI / 4) - theta ;

        telemetry.addData("theta", theta);

        telemetry.addData("orientation", orientation);
        telemetry.update();

        // Pass that angle through a pair of wave functions to get the power for each corresponding pair of parallel wheels
        negative_power = 1 * (joystick_power * Math.sin(orientation));
        positive_power = (orientation != 0) ? 1 * (joystick_power * Math.cos(orientation)) :
                negative_power;

        // This is all we need to actually move the robot, method decs in Core.java

        if (gamepad1.x == true){
            move(2 * positive_power, 2 * negative_power, 2 * rot_power);
        }
        else {
            move(positive_power, negative_power, rot_power);
        }
    }
}
