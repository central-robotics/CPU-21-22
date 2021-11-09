package org.firstinspires.ftc.teamcode.autonomous.localization;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;

import java.util.Vector;

public class Localizer {
    private Vision vision;
    private Encoder encoder;

    public void initializeLocalizer()
    {
        vision = new Vision();
        vision.initializeLocalizer();

        vision.targets.activate();
    }

    public Location getRobotPosition()
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
            Orientation orientation = Orientation.getOrientation(vision.location, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
            Location location = new Location();
            location.orientation = orientation;
            location.translation = translation;
            return location;
        }

       return null;
    }

    class Location
    {
        VectorF translation;
        Orientation orientation;
    }
}
