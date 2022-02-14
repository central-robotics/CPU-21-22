package org.firstinspires.ftc.teamcode.autonomous.actions;

import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.autonomous.AutonCore;
import org.firstinspires.ftc.teamcode.autonomous.Constants;
import org.firstinspires.ftc.teamcode.autonomous.actions.util.ObjectDetector;
import org.firstinspires.ftc.teamcode.autonomous.control.PID;
import org.firstinspires.ftc.teamcode.autonomous.hardware.Hardware;
import org.firstinspires.ftc.teamcode.autonomous.instructions.Instructions;
import org.firstinspires.ftc.teamcode.autonomous.localization.Localization;
import org.firstinspires.ftc.teamcode.autonomous.localization.Position;
import org.firstinspires.ftc.teamcode.autonomous.nav.path.LinearPath;
import org.firstinspires.ftc.teamcode.autonomous.nav.path.Path;
import org.firstinspires.ftc.teamcode.autonomous.vision.Vuforia;
import org.firstinspires.ftc.teamcode.autonomous.nav.Navigation;

public class PlaceCubeAction extends Action {
    private final Navigation navigation;

    private double armPos;
    private double prevArmPos;

    public PlaceCubeAction(int index, Navigation navigation) {
        super(index);
        this.navigation = navigation;
    }

    @Override
    public void execute(Hardware hardware, Localization localization, Vuforia vuforia, ObjectDetector detector) {
        ObjectDetector.BarcodeLocation location = detector.calculateState();
        PID slidePID = new PID(new PIDCoefficients(0.008, 0.0001, 0));

        double slideLevel = 5;

        switch (location)
        {
            case LEFT:
                if (Constants.IS_BLUE_TEAM)
                    slideLevel = 500;
                else
                    slideLevel = 0;
                break;
            case CENTER:
                slideLevel = 200;
                break;
            case RIGHT:
                if (Constants.IS_BLUE_TEAM)
                    slideLevel = 0;
                else
                    slideLevel = 500;
                break;
            default:
                break;
        }

        Position pos;

        double x;

        switch ((int) slideLevel)
        {
            case 0:
                x = 750;
                break;
            case 200:
                x = 650;
                break;
            case 500:
                x = 550;
                break;
            default:
                x = 650;
                break;
        }

        if (Constants.IS_BLUE_TEAM)
        {

            if (Constants.IS_LEFT_OPMODE)
            {
                pos = new Position(x, 1830, Constants.CURRENT_INITIAL_THETA - 0.5);

            } else
            {
                pos = new Position(x, 1234, Constants.CURRENT_INITIAL_THETA + 0.5);
            }
        } else
        {

            if (Constants.IS_LEFT_OPMODE)
            {
                pos = new Position(x, 1234, Constants.CURRENT_INITIAL_THETA - 0.5);
            } else
            {
                pos = new Position(x, 1830, Constants.CURRENT_INITIAL_THETA + 0.5);
            }
        }

        if (!Constants.IS_BLUE_TEAM)
            pos.x *= -1;

        Instructions.navigation.drive.driveToTarget(pos);

        moveToLevel(slideLevel, hardware, slidePID);
    }



    private void moveToLevel(double ticks, Hardware hardware, PID pid)
    {
        armPos = hardware.armMotor.getCurrentPosition();

        ElapsedTime time = new ElapsedTime();

        while (armPos < ticks - 20)
        {
            if (armPos > 200) {
                hardware.boxServo.setPosition(0.50);
            }

            double error = ticks - armPos;

            hardware.armMotor.setPower(0.2 * pid.getOutput(error, 0));

            armPos = hardware.armMotor.getCurrentPosition();

            AutonCore.telem.addData("TARGET POS", ticks);
            AutonCore.telem.addData("CURRENT POS", armPos);
            AutonCore.telem.update();
        }

        time.reset();

        hardware.boxServo.setPosition(1);

        while (time.milliseconds() < 2000) {

        }


        armPos = hardware.armMotor.getCurrentPosition();

        while (armPos > 20)
        {
            if (armPos < 100)
                break;

            if (ticks == 0)
                break;

            if (armPos < 300)
                hardware.boxServo.setPosition(0.7);

            double error = 20 - armPos;

            hardware.armMotor.setPower(0.2 * pid.getOutput(error, 0));

            armPos = hardware.armMotor.getCurrentPosition();
        }
    }
}
