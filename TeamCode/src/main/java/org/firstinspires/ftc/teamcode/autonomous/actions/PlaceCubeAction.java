package org.firstinspires.ftc.teamcode.autonomous.actions;

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
    public void execute(Hardware hardware, Localization localization, Vuforia vuforia) {

        ObjectDetector objectDetector = new ObjectDetector(hardware, vuforia);

        Position position = new Position();

        //SHIPPING HUB COORD.
        position.x = 1000;
        position.y = 1000;

        Position currentPosition = localization.getRobotPosition();

        if (recognition == null)
        {
            position.x = currentPosition.x + Constants.DIST_TO_BARCODE;
            position.y = currentPosition.y;
        }
        double leftDist = recognition.getLeft();

        ArmLevel armLevel = ArmLevel.MIDDLE;

        if (leftDist > 300)
        {
            //Right case
            armLevel = ArmLevel.TOP;
        } else if (leftDist > 200)
        {
            //Center case
            armLevel = ArmLevel.MIDDLE;
        } else if (leftDist > 100)
        {
            //Left case
            armLevel = ArmLevel.BOTTOM;
        }

        navigation.drive.driveToTarget(position);

        switch (armLevel)
        {
            case BOTTOM:
                break;
            case MIDDLE:
                break;
            case TOP:
                break;
        }
    }

    private void moveArm(ArmLevel armLevel, Hardware hardware)
    {
        double targetPos = 700;

        switch (armLevel) {
            case BOTTOM:
                targetPos = 400;
                break;
            case MIDDLE:
                targetPos = 700;
                break;
            case TOP:
                targetPos = 1000;
                break;
        }

        PIDCoefficients armCoefficients = new PIDCoefficients(-0.05, 0, 0);
        PID pid = new PID(armCoefficients);

        while (Math.abs(hardware.intakeMotor.getCurrentPosition()) > targetPos - 20
                && Math.abs(hardware.intakeMotor.getCurrentPosition()) < targetPos + 20)
        {

        }
    }

    private enum ArmLevel
    {
        BOTTOM,
        MIDDLE,
        TOP,
        CAP
    }
}
