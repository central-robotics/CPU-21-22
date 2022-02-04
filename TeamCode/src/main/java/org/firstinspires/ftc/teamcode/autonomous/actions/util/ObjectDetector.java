package org.firstinspires.ftc.teamcode.autonomous.actions.util;

import com.vuforia.Trackable;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.autonomous.Constants;
import org.firstinspires.ftc.teamcode.autonomous.actions.PlaceCubeAction;
import org.firstinspires.ftc.teamcode.autonomous.hardware.Hardware;
import org.firstinspires.ftc.teamcode.autonomous.vision.Vuforia;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvPipeline;
import org.outoftheboxrobotics.tensorflowapi.ImageClassification.TFICBuilder;
import org.outoftheboxrobotics.tensorflowapi.ImageClassification.TensorImageClassifier;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

public class ObjectDetector {
    private Vuforia vuforia;
    private OpenCvCamera camera;
    private ElementPipeline pipeline;
    private Hardware hardware;
    private TensorImageClassifier imageClassifier;

    public ObjectDetector(Hardware hardware, Vuforia vuforia)
    {
        this.hardware = hardware;
        this.vuforia = vuforia;
        pipeline = new ElementPipeline();

        initializeObjectDetector();
    }

    private void initializeObjectDetector()
    {
        try {
            imageClassifier = new TFICBuilder(hardware.map, "model.tflite", "NoTeamElement", "TeamElement").setQuantized(true).build();
        } catch (IOException e) {
            e.printStackTrace();
        }

        //OpenCvCamera camera = OpenCvCameraFactory.getInstance().createVuforiaPassthrough(vuforia.vuforiaLocalizer, vuforia.parameters);
        camera = hardware.openCvCamera;
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(1280, 720);
                camera.setPipeline(pipeline);
            }

            @Override
            public void onError(int i) {

            }
        });
    }

    //To allow the robot to figure out the position of any element without training an AI model, we will figure out which barcode we can't see.
    public BarcodeLocation getRecognition()
    {
        Mat mat = pipeline.getMat();

        Mat[] submats = new Mat[3];

        Mat rawLeftMat = mat.submat(new Rect(0, 0, Constants.WEBCAM_SECTION_WIDTH, Constants.WEBCAM_HEIGHT));
        Mat leftMat = new Mat();
        Imgproc.resize(rawLeftMat, leftMat, new Size(224, 224));

        Mat rawCenterMat = mat.submat(new Rect(Constants.WEBCAM_SECTION_WIDTH, 0, Constants.WEBCAM_SECTION_WIDTH, Constants.WEBCAM_HEIGHT));
        Mat centerMat = new Mat();
        Imgproc.resize(rawCenterMat, centerMat, new Size(224, 224));

        Mat rawRightMat = mat.submat(new Rect(Constants.WEBCAM_SECTION_WIDTH, 0, Constants.WEBCAM_SECTION_WIDTH, Constants.WEBCAM_HEIGHT));
        Mat rightMat = new Mat();
        Imgproc.resize(rawRightMat, rightMat, new Size(224, 224));

        submats[0] = leftMat;
        submats[1] = centerMat;
        submats[2] = rightMat;

        float highestConfidence = 0;
        int elementIndex = 1;

        for (int i = 0; i < submats.length; i++)
        {
            float confidence = getConfidence(imageClassifier.recognize(submats[i]));

            if (confidence > highestConfidence)
            {
                highestConfidence = confidence;
                elementIndex = i;
            }
        }

        return BarcodeLocation.values()[elementIndex];
    }

    private float getConfidence(List<TensorImageClassifier.Recognition> output) {
        if (output.size() < 2) {
            return -1;
        }
        return output.get(0).getTitle().equals("TeamElement") ? output.get(0).getConfidence() : output.get(1).getConfidence();
    }

    public enum BarcodeLocation
    {
        LEFT, CENTER, RIGHT;
    }
}
