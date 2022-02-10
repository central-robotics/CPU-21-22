package org.firstinspires.ftc.teamcode.autonomous.actions;

import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.autonomous.AutonCore;
import org.firstinspires.ftc.teamcode.autonomous.Constants;
import org.firstinspires.ftc.teamcode.autonomous.actions.util.ObjectDetector;
import org.firstinspires.ftc.teamcode.autonomous.control.PID;
import org.firstinspires.ftc.teamcode.autonomous.hardware.Hardware;
import org.firstinspires.ftc.teamcode.autonomous.localization.Localization;
import org.firstinspires.ftc.teamcode.autonomous.localization.Position;
import org.firstinspires.ftc.teamcode.autonomous.vision.Vuforia;
import org.firstinspires.ftc.teamcode.autonomous.nav.Navigation;

public class PlaceCubeAction extends Action {
    private final Navigation navigation;

    public PlaceCubeAction(int index, Navigation navigation) {
        super(index);
        this.navigation = navigation;
    }

    @Override
    public void execute(Hardware hardware, Localization localization, Vuforia vuforia, ObjectDetector detector) {
        ObjectDetector.BarcodeLocation location = detector.calculateState();

        PID slidePID = new PID(new PIDCoefficients(0.3, 0, 0));

        double slideLevel = 5;

        switch (location)
        {
            case LEFT:
                if (Constants.IS_BLUE_TEAM)
                    slideLevel = 600;
                else
                    slideLevel = 0;
                break;
            case CENTER:
                slideLevel = 250;
                break;
            case RIGHT:
                if (Constants.IS_BLUE_TEAM)
                    slideLevel = 0;
                else
                    slideLevel = 600;
                break;
            default:
                break;
        }
        if (Constants.IS_BLUE_TEAM)
        {
            if (Constants.IS_LEFT_OPMODE)
            {
                Position pos = new Position(700, 1830, Constants.CURRENT_INITIAL_THETA);

            } else
            {
                Position pos = new Position(700, 1234, Constants.CURRENT_INITIAL_THETA);
            }
        } else
        {
            if (Constants.IS_LEFT_OPMODE)
            {
                Position pos = new Position(700, 1234, Constants.CURRENT_INITIAL_THETA);
            } else
            {
                Position pos = new Position(700, 1830, Constants.CURRENT_INITIAL_THETA);
            }

            while (Math.abs(hardware.armMotor.getCurrentPosition()) < slideLevel) {
                hardware.armMotor.setPower(-0.8);
                AutonCore.telem.addData("ticks", hardware.armMotor.getCurrentPosition());
                AutonCore.telem.update();

            }

            hardware.armMotor.setPower(0);

            while (hardware.boxServo.getPosition() < 0.83)
                hardware.boxServo.setPosition(0.85);


            while (Math.abs(hardware.armMotor.getCurrentPosition()) > 50)
                hardware.armMotor.setPower(0.3);

            hardware.boxServo.setPosition(0.61);


        }
    }

    private void moveToLevel(double ticks, Hardware hardware, PID pid)
    {
        double pos = hardware.armMotor.getCurrentPosition();

        while (Math.abs(pos) < ticks - 5)
        {
            hardware.armMotor.setPower(pid.getOutput(ticks - pos, 0));
            pos = hardware.armMotor.getCurrentPosition();
        }

        ElapsedTime time = new ElapsedTime();

        while (time.milliseconds() < )

        while (Math.abs(pos) > 30)
        {
            hardware.armMotor.setPower(pid.getOutput(ticks - pos, 0));
            pos = hardware.armMotor.getCurrentPosition();
        }
    }
}
