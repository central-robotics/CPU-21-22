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
            case RIGHT:
                if (Constants.IS_BLUE_TEAM)
                    slideLevel = 565;
                else
                    slideLevel = 70;
                break;
            case CENTER:
                slideLevel = 300;
                break;
            case LEFT:
                if (Constants.IS_BLUE_TEAM)
                    slideLevel = 70;
                else
                    slideLevel = 565;
                break;
            default:
                break;
        }

        Position pos;

        double x;
        double y;

        if (Constants.IS_BLUE_TEAM)
        {
            if (Constants.IS_LEFT_OPMODE) //BLUE WAREHOUSE
            {
                switch ((int) slideLevel)
                {
                    case 70:
                        x = 975;
                        y = 1565;
                        break;
                    case 300:
                        x = 795;
                        y = 1695;
                        break;
                    case 565:
                        x = 785;
                        y = 1785;
                        break;
                    default:
                        x = 800;
                        y = 1760;
                        break;
                }
            } else
            {
                switch ((int) slideLevel) //BLUE CAROUSEL
                {
                    case 70:
                        x = 975;
                        y = 1565;
                        break;
                    case 300:
                        x = 795;
                        y = 1695;
                        break;
                    case 565:
                        x = 785;
                        y = 1785;
                        break;
                    default:
                        x = 800;
                        y = 1760;
                        break;
                }
            }
        } else
        {
            if (Constants.IS_LEFT_OPMODE) //RED CAROUSEL
            {
                switch ((int) slideLevel)
                {
                    case 70:
                        x = 775;
                        y = 1074;
                        break;
                    case 300:
                        x = 745;
                        y = 1104;
                        break;
                    case 565:
                        x = 715;
                        y = 1134;
                        break;
                    default:
                        x = 800;
                        y = 1134;
                        break;
                }
            } else //RED WAREHOUSE
            {
                switch ((int) slideLevel)
                {
                    case 70:
                        x = 975;
                        y = 1565;
                        break;
                    case 300:
                        x = 795;
                        y = 1695;
                        break;
                    case 565:
                        x = 785;
                        y = 1785;
                        break;
                    default:
                        x = 800;
                        y = 1760;
                        break;
                }
            }
        }




        if (Constants.IS_BLUE_TEAM)
        {

            if (Constants.IS_LEFT_OPMODE)
            {
                pos = new Position(x, y, Constants.CURRENT_INITIAL_THETA - 0.65);

            } else
            {
                pos = new Position(x, y, Constants.CURRENT_INITIAL_THETA + 0.65);
            }
        } else
        {

            if (Constants.IS_LEFT_OPMODE)
            {
                pos = new Position(x, y, Constants.CURRENT_INITIAL_THETA - 0.65);
            } else
            {
                pos = new Position(x, y, Constants.CURRENT_INITIAL_THETA + 0.65);
            }
        }

        if (!Constants.IS_BLUE_TEAM)
            pos.x *= -1;

        Instructions.navigation.drive.driveToTarget(pos);

        moveToLevel(slideLevel, hardware, slidePID, localization);
    }



    private void moveToLevel(double ticks, Hardware hardware, PID pid, Localization localization)
    {
        armPos = hardware.armMotor.getCurrentPosition();

        ElapsedTime time = new ElapsedTime();

        boolean begunCorrect = false;

        hardware.setAllMotorPowers(0);

        while (armPos < ticks - 20)
        {
            if (armPos > 60 && ticks != 0 && !begunCorrect) {
                begunCorrect = true;
                new Thread(() -> {
                    hardware.boxServo.setPosition(0.35);
                }).start();
            }

            double error = ticks - armPos;

            hardware.armMotor.setPower(0.2 * pid.getOutput(error, 0));

            armPos = hardware.armMotor.getCurrentPosition();

            AutonCore.telem.addData("TARGET POS", ticks);
            AutonCore.telem.addData("CURRENT POS", armPos);
            AutonCore.telem.update();
        }

        time.reset();

        hardware.setAllMotorPowers(0);

        while (time.milliseconds() < 500)
        {
            //Nothing
        }

        new Thread(() -> {
            hardware.boxServo.setPosition(1);
        }).start();

        time.reset();

        while (time.milliseconds() < 1500)
        {
            //Nothing
        }


        while (hardware.boxServo.getPosition() < 0.95)
        {
            //Do nothing
        }

        Position newPos = localization.getRobotPosition();

        if (Constants.IS_BLUE_TEAM)
        {

            if (Constants.IS_LEFT_OPMODE)
            {
                newPos.x -= 200;
                newPos.y += 200;

            } else
            {
                newPos.x -= 200;
                newPos.y -= 0;
            }

        } else
        {

            if (Constants.IS_LEFT_OPMODE)
            {
                newPos.x += 200;
                newPos.y -= 0;
            } else
            {
                newPos.x -= 200;
                newPos.y += 200;
            }
        }


        Instructions.navigation.drive.driveToTarget(newPos);


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
