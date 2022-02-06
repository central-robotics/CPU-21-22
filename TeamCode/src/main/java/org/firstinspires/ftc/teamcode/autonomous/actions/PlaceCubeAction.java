package org.firstinspires.ftc.teamcode.autonomous.actions;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.MotionDetection;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.autonomous.Constants;
import org.firstinspires.ftc.teamcode.autonomous.actions.util.ObjectDetector;
import org.firstinspires.ftc.teamcode.autonomous.control.PID;
import org.firstinspires.ftc.teamcode.autonomous.hardware.Hardware;
import org.firstinspires.ftc.teamcode.autonomous.localization.Localization;
import org.firstinspires.ftc.teamcode.autonomous.localization.Position;
import org.firstinspires.ftc.teamcode.autonomous.vision.Vuforia;
import org.firstinspires.ftc.teamcode.autonomous.waypoint.Navigation;

public class PlaceCubeAction extends Action {
    private Navigation navigation;

    public PlaceCubeAction(int index, Navigation navigation) {
        super(index);
        this.navigation = navigation;
    }

    @Override
    public void execute(Hardware hardware, Localization localization, Vuforia vuforia, ObjectDetector detector) {
        ObjectDetector.BarcodeLocation location = detector.calculateState();

        double slideLevel = 5;

        switch (location)
        {
            case LEFT:
                if (Constants.IS_BLUE_TEAM)
                    slideLevel = 10000;
                else
                    slideLevel = 0;
                break;
            case CENTER:
                slideLevel = 5000;
                break;
            case RIGHT:
                if (Constants.IS_BLUE_TEAM)
                    slideLevel = 0;
                else
                    slideLevel = 10000;
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

            while (Math.abs(hardware.armMotor.getCurrentPosition()) < slideLevel)
                hardware.armMotor.setPower(0.4);
            hardware.armMotor.setPower(0.1);


        }
    }
}
