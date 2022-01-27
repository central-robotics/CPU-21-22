package org.firstinspires.ftc.teamcode.autonomous.actions.util;

import com.vuforia.Rectangle;

import org.firstinspires.ftc.teamcode.autonomous.Constants;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class ElementPipeline extends OpenCvPipeline {
    private int elementIndex = 0; //0 = LEFT, 1 = CENTER, 2 = RIGHT

    @Override
    public Mat processFrame(Mat mat) {
        Mat[] submats = new Mat[3];

        //Rectangles have been laid out here for visual context.
        Rect rect1 = new Rect();
        rect1.width = mat.width() / 3;
        rect1.height = mat.height();
        rect1.x = 0;
        rect1.y = 0;

        submats[0] = mat.submat(rect1);

        Rect rect2 = new Rect();
        rect2.width = mat.width() / 3;
        rect2.height = mat.height();
        rect2.x = mat.width() / 3;
        rect2.y =  0;

        submats[1] = mat.submat(rect2);

        Rect rect3 = new Rect();
        rect3.width = mat.width() / 3;
        rect3.height = mat.height();
        rect3.x = (mat.width() * 2) / 3;
        rect3.y = 0;

        submats[2] = mat.submat(rect3);

        int lowestDifference = 360; //Hue that is closest to target hue. Probably the target? I don't know at this point. Screw color spaces.
        for (int i = 0; i < submats.length; i++ )
        {
            Mat destination = new Mat();
            Mat extracted = new Mat();

            Imgproc.cvtColor(submats[i], destination, Imgproc.COLOR_RGB2HSV); //Transforms RGB TO HSV
            Core.extractChannel(destination, extracted, 0); //Extracts hue component from HSV.

            int avgHue = (int) Core.mean(extracted).val[0];

            if (Math.abs(Constants.HUE_TARGET - avgHue) < lowestDifference) {
                lowestDifference = Math.abs(Constants.HUE_TARGET - avgHue);
                elementIndex = i;
            }

        }

        return null;
    }

    public int getElementIndex()
    {
        return elementIndex;
    }
}
