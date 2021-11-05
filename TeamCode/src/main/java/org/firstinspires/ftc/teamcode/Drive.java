package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp
public class Drive extends Core {
    double positive_power, negative_power;
    double joystick_x, joystick_y, joystick_pivot, joystick_power;
    double orientation, pivot_target;
    Orientation gyro_angles;

    public void loop(){

        // Get all the info we from the gamepad
        joystick_y = gamepad1.left_stick_y;
        joystick_x = (gamepad1.left_stick_x == 0) ? 0.000001 :
                gamepad1.left_stick_x;
        joystick_pivot = gamepad1.right_stick_x;
        joystick_power = Math.sqrt(Math.pow(joystick_x, 2) + Math.pow(joystick_y, 2));

        gyro_angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);

        // Turn the joystick coordinates into an angle in radians
        orientation = (joystick_x > 0) ? (Math.atan(-joystick_y / joystick_x) - Math.PI / 4) - gyro_angles.firstAngle :
                (Math.atan(-joystick_y / joystick_x) + Math.PI - Math.PI / 4) - gyro_angles.firstAngle;

        // Turn that angle into a wave function for the _power of each pair of parallel wheels
        negative_power = 0.6 * (joystick_power * Math.sin(orientation));
        positive_power = (orientation != 0) ? 0.6 * (joystick_power * Math.cos(orientation)) :
                negative_power;

        // This is all we need to actually move the robot (!!)
        if (joystick_pivot != 0)
        {
            pivot(0.3 * joystick_pivot);
        }
        else
            move(positive_power, negative_power);
    }
}
