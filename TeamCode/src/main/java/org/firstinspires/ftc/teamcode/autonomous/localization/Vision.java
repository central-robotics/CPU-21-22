package org.firstinspires.ftc.teamcode.autonomous.localization;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.teamcode.autonomous.Constants;
import org.firstinspires.ftc.teamcode.autonomous.vision.Vuforia;

public class Vision {
    private final Vuforia vuforia;
    private OpenGLMatrix location;
    public Boolean targetVisible = false;

    public Vision(Vuforia vuforia)
    {
        this.vuforia = vuforia;
    }

    public Position getRobotPosition()
    {
        for (VuforiaTrackable trackable : vuforia.trackables) {
            if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                //Target is visible

                targetVisible = true;

                // getUpdatedRobotLocation() will return null if no new information is available since
                // the last time that call was made, or if the trackable is not currently visible.
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    location = robotLocationTransform;
                }
                break;
            }
        }

        if (targetVisible) {
            VectorF translation = location.getTranslation();
            Orientation orientation = Orientation.getOrientation(location, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS);

            Position position = new Position();

            position.x = translation.get(0) / 25.4;
            position.y = translation.get(1) / 25.4;
            position.t = orientation.thirdAngle - Constants.INIT_THETA;

            targetVisible = false;

            return position;
        }

        targetVisible = false;

        return null;
    }
}
