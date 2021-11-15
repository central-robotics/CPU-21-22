package org.firstinspires.ftc.teamcode.autonomous.localization;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.teamcode.autonomous.hardware.Gyro;
import org.firstinspires.ftc.teamcode.autonomous.hardware.HardwareUtil;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.Vector;

public class Localizer {
    public Position robotPos;

    private Vision vision;
    private Encoder encoder;
    private float previousTheta;

    public void initializeLocalizer()
    {
        encoder = new Encoder();
        encoder.initializeLocalizer();
        vision = new Vision();
        vision.initializeLocalizer();
        vision.targets.activate();
    }

    private Position getVisionPos()
    {
        for (VuforiaTrackable trackable : vision.trackables) {
            if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                //Target is visible

                vision.targetVisible = true;

                // getUpdatedRobotLocation() will return null if no new information is available since
                // the last time that call was made, or if the trackable is not currently visible.
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    vision.location = robotLocationTransform;
                }
                break;
            }
        }

        if (vision.targetVisible)
        {
            VectorF translation = vision.location.getTranslation();
            Orientation orientation = Orientation.getOrientation(vision.location, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS);

            Position position = new Position();

            position.x = translation.get(0) / 25.4f;
            position.y = translation.get(1) / 25.4f;
            position.t = orientation.thirdAngle;

            vision.targetVisible = false;

            return position;
        }

        vision.targetVisible = false;

        return null;
    }

    private Position getEncoderPos()
    {
        ArrayList<Float> wheelDisplacements = encoder.calculateDisplacements();

        float displacementTotal = 0.0f;

        for (float disp : wheelDisplacements)
        {
            displacementTotal += disp;
        }

        float avgDisp = displacementTotal / 4.0f;

        ArrayList<Float> actualDisplacement = new ArrayList<>();

        for (float disp : wheelDisplacements)
        {
            actualDisplacement.add(disp - avgDisp);
        }

        float robotDeltaX =  (float) ((
                actualDisplacement.get(0) +
                actualDisplacement.get(1) -
                actualDisplacement.get(2) -
                (actualDisplacement.get(3))) /
                (2 * Math.sqrt(2)));

        float robotDeltaY =  (float) ((
                actualDisplacement.get(0) -
                actualDisplacement.get(1) -
                actualDisplacement.get(2) +
                actualDisplacement.get(3)) /
                (2 * Math.sqrt(2)));

        //NOT CONFIDENT ON ROBOT DIAMETER. USING IMU UNTIL ISSUE IS FIXED

        //float rotation = (float) ((avgDisp / 57.757f /*CIRCUMFERENCE OF WHEEL DIAGONALS*/) * (2 * Math.sqrt(2)));

        float rotation = Gyro.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS).firstAngle;

        float theta = previousTheta - rotation;
        previousTheta = theta;

        //translate robot to field coordinates
        float fieldDeltaX = (float) (robotDeltaX * Math.cos(theta) - robotDeltaY * Math.sin(theta));
        float fieldDeltaY = (float) (robotDeltaY * Math.cos(theta) + robotDeltaX * Math.sin(theta));

        Position position = new Position();
        position.x = fieldDeltaX;
        position.y = fieldDeltaY;
        position.t = theta;

        return position;
    }

    public Position updatePosition()
    {
        Position encoderPos = getEncoderPos();
        Position visionPos = getVisionPos();

        if (visionPos != null)
        {
            robotPos = visionPos;
            return visionPos;
        } else
        {
            robotPos = encoderPos;
            return encoderPos;
        }
    }

    public class Position //Position of robot on field (x, y, theta)
    {
        public Float x, y, t;
    }
}
