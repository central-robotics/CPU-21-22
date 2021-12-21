package org.firstinspires.ftc.teamcode.autonomous.vision;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.autonomous.Constants;
import org.firstinspires.ftc.teamcode.autonomous.hardware.Hardware;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XZY;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

public class Vuforia {
    private Hardware hardware;

    public VuforiaLocalizer vuforiaLocalizer; //Vuforia instance
    public VuforiaTrackables targets; //Vuforia image
    public List<VuforiaTrackable> trackables;

    private static final float mmTargetHeight = 152.4f;
    private static final float halfField = 1828.8f;
    private static final float halfTile = 304.8f;
    private static final float oneAndHalfTile = 914.4f;

    public Vuforia(Hardware hardware)
    {
        this.hardware = hardware;
        initializeVuforia();
    }

    private void initializeVuforia()
    {
        VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters();
        params.vuforiaLicenseKey = Constants.VUFORIA_KEY;
        params.cameraName = hardware.camera;
        params.useExtendedTracking = false;

        vuforiaLocalizer = ClassFactory.getInstance().createVuforia(params);
        targets = vuforiaLocalizer.loadTrackablesFromAsset("FreightFrenzy");
        trackables = new ArrayList<>();
        trackables.addAll(targets);

        identifyTarget(0, "Blue Storage", -halfField, oneAndHalfTile, mmTargetHeight, 90, 0, 90);
        identifyTarget(1, "Blue Alliance Wall", halfTile, halfField, mmTargetHeight, 90, 0, 0);
        identifyTarget(2, "Red Storage", -halfField, -oneAndHalfTile, mmTargetHeight, 90, 0, 90);
        identifyTarget(3, "Red Alliance Wall", halfTile, -halfField, mmTargetHeight, 90, 0, 180);

        identifyObject(4, "CPU Element");

        OpenGLMatrix cameraLocation = OpenGLMatrix //We need to describe where the camera is on the robot.
                .translation(4.0f /*Forward displacement from center*/, 0.0f /*Left displacement from center*/, 0f  /*Vertical displacement from ground*/)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XZY, DEGREES, 90, 90, 0));

        for (VuforiaTrackable trackable : trackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setCameraLocationOnRobot(params.cameraName, cameraLocation);
        }
    }

    private void identifyTarget(int targetIndex, String targetName, float dx, float dy, float dz, float rx, float ry, float rz) {
        VuforiaTrackable aTarget = targets.get(targetIndex);
        aTarget.setName(targetName);
        aTarget.setLocation(OpenGLMatrix.translation(dx, dy, dz)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, rx, ry, rz)));
    }

    private void identifyObject(int targetIndex, String targetName)
    {
        VuforiaTrackable target = targets.get(targetIndex);
        target.setName(targetName);
    }

}
