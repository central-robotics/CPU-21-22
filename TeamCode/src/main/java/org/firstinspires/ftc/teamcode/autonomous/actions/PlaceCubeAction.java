package org.firstinspires.ftc.teamcode.autonomous.actions;

import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.autonomous.AutonCore;
import org.firstinspires.ftc.teamcode.autonomous.Constants;
import org.firstinspires.ftc.teamcode.autonomous.actions.util.ObjectDetector;
import org.firstinspires.ftc.teamcode.shared.control.PID;
import org.firstinspires.ftc.teamcode.autonomous.hardware.Hardware;
import org.firstinspires.ftc.teamcode.autonomous.instructions.Instructions;
import org.firstinspires.ftc.teamcode.autonomous.localization.Localization;
import org.firstinspires.ftc.teamcode.autonomous.localization.Position;
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

        ObjectDetector.BarcodeLocation location = detector.calculateStateUsingColor();
        PID slidePID = new PID(new PIDCoefficients(0.008, 0.0001, 0));

        double slideLevel = 5;

        switch (location)
        {
            case LEFT:
                slideLevel = 70;
                break;
            case CENTER:
                slideLevel = 110;
                break;
            case RIGHT:
                slideLevel = 440;
                break;
            default:
                slideLevel = 260;
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
                        y = 1715;
                        break;
                    case 110:
                        x = 795;
                        y = 1845;
                        break;
                    case 440:
                        x = 785;
                        y = 1935;
                        break;
                    default:
                        x = 795;
                        y = 1695;
                        break;
                }
            } else
            {
                switch ((int) slideLevel) //BLUE CAROUSEL
                {
                    case 70:
                        x = 945;
                        y = 1395;
                        break;
                    case 110:
                        x = 795;
                        y = 1330;
                        break;
                    case 440:
                        x = 745;
                        y = 1330;
                        break;
                    default:
                        x = 745;
                        y = 1254;
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
                        x = 900;
                        y = 1185;
                        break;
                    case 110:
                        x = 765;
                        y = 1134;
                        break;
                    case 440:
                        x = 715;
                        y = 1134;
                        break;
                    default:
                        x = 745;
                        y = 1104;
                        break;
                }
            } else //RED WAREHOUSE
            {
                switch ((int) slideLevel)
                {
                    case 70:
                        x = 820;
                        y = 1690;
                        break;
                    case 110:
                        x = 780;
                        y = 1730;
                        break;
                    case 440:
                        x = 740;
                        y = 1790;
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
                new Thread(() ->
                {
                    hardware.boxServo.setPosition(0.42);
                }).start();
                // could also break after only one run due to thread
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

        hardware.armMotor.setPower(0.001);

        new Thread(() -> {
            hardware.boxServo.setPosition(0.02);
        }).start();

        while (time.milliseconds() < 600)
        {

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

        time.reset();

        new Thread(() ->
        {
            hardware.boxServo.setPosition(0.68);
        }).start();


        while (time.milliseconds() < 500)
        {
            //Nothing
        }


        armPos = hardware.armMotor.getCurrentPosition();

        while (armPos > 20)
        {
            if (armPos < 100)
                break;

            if (ticks == 0)
                break;

            double error = 20 - armPos;

            hardware.armMotor.setPower(0.2 * pid.getOutput(error, 0));

            armPos = hardware.armMotor.getCurrentPosition();
        }
    }
}
