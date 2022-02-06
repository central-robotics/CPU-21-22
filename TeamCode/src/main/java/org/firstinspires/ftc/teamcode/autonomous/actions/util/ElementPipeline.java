package org.firstinspires.ftc.teamcode.autonomous.actions.util;

import com.vuforia.Rectangle;

import org.firstinspires.ftc.teamcode.autonomous.Constants;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class ElementPipeline extends OpenCvPipeline {
    private final int elementIndex = 0; //0 = LEFT, 1 = CENTER, 2 = RIGHT
    private Mat mat;

    @Override
    public Mat processFrame(Mat mat) {
        this.mat = mat;
        return mat;
    }

    public Mat getMat()
    {
        return mat;
    }
}
