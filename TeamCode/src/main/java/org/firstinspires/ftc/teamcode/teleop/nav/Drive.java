package org.firstinspires.ftc.teamcode.teleop.nav;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.teleop.TeleOpConstants;
import org.firstinspires.ftc.teamcode.teleop.TeleOpHardware;

public final class Drive {
    public static void moveRobot(Gamepad gamepad, TeleOpHardware hardware, boolean turboEnabled)
    {
        double joystickX, joystickY, rot, power, theta, orientation, pos, neg;

        joystickY = gamepad.left_stick_y > 0 ? Math.pow(gamepad.left_stick_y, 2) :
                -Math.pow(gamepad.left_stick_y, 2);

        joystickX= (gamepad.left_stick_x == 0) ? 0.000001 :
                (gamepad.left_stick_x > 0 ? Math.pow(gamepad.left_stick_x, 2) :
                        -Math.pow(gamepad.left_stick_x, 2));

        rot = 0.7 * (gamepad.right_stick_x);

        power = Math.sqrt(Math.pow(joystickX, 2) + Math.pow(joystickY, 2));

        theta = hardware.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle
                + TeleOpConstants.imuHeading;

        orientation = (joystickX > 0) ? (Math.atan(-joystickY / joystickX) - Math.PI / 4)  - theta :
                (Math.atan(-joystickY/joystickX) + Math.PI - Math.PI / 4) - theta ;

        neg = (power * Math.sin(orientation));
        pos = (orientation != 0) ? (power * Math.cos(orientation)) :
                neg;

        if (!turboEnabled)
            setPowers(pos, neg, rot, hardware);
        else
            setPowers(2 * pos, 2 * neg, 2 * rot, hardware);

    }

    public static void setPowers(double pos, double neg, double rot, TeleOpHardware hardware)
    {
        hardware.lf.setPower(.6* (-pos-rot));
        hardware.rf.setPower(.6 * (neg-rot));
        hardware.lb.setPower(.6 * (-neg-rot));
        hardware.rb.setPower(.6 * (pos-rot));
    }
}
