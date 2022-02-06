package org.firstinspires.ftc.teamcode.autonomous.actions.util;

import com.vuforia.Trackable;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.autonomous.AutonCore;
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
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.outoftheboxrobotics.tensorflowapi.ImageClassification.TFICBuilder;
import org.outoftheboxrobotics.tensorflowapi.ImageClassification.TensorImageClassifier;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

public class ObjectDetector {
    private final Vuforia vuforia;
    public BarcodeLocation location;
    private OpenCvCamera camera;
    private final ElementPipeline pipeline;
    private final Hardware hardware;
    private TensorImageClassifier imageClassifier;
    private volatile Boolean cameraReady = false;

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
        camera.setPipeline(pipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);

                cameraReady = true;
                AutonCore.telem.addLine("Webcam ready");
                AutonCore.telem.update();
            }

            @Override
            public void onError(int i) {

            }
        });
    }

    public BarcodeLocation calculateState() {
        camera.stopStreaming();
        camera.closeCameraDevice();
        BarcodeLocation teamElementLocation;
        System.out.println("handleMat")
        ;
        if (imageClassifier == null || pipeline.getMat() == null) {
            return null;
        }

        Mat mat = pipeline.getMat();
        int width = mat.width(), height = mat.height();
        // todo: these threshold values probably need to be changed
        Mat rawLeftMat = mat.submat(new Rect(0, 0, Constants.WEBCAM_SECTION_WIDTH, Constants.WEBCAM_HEIGHT));
        Mat leftMat = new Mat();
        Imgproc.resize(rawLeftMat, leftMat, new Size(224, 224));

        List<TensorImageClassifier.Recognition> leftOutput = imageClassifier.recognize(leftMat);

        float leftConfidence = getConfidence(leftOutput);

        if (leftConfidence > 0.5) {
            teamElementLocation = BarcodeLocation.LEFT;
            return teamElementLocation;
        }

        Mat rawCenterMat = mat.submat(new Rect(Constants.WEBCAM_SECTION_WIDTH, 0, Constants.WEBCAM_SECTION_WIDTH, Constants.WEBCAM_HEIGHT));
        Mat centerMat = new Mat();

        Imgproc.resize(rawCenterMat, centerMat, new Size(224, 224));

        List<TensorImageClassifier.Recognition> centerOutput = imageClassifier.recognize(centerMat);
        float centerConfidence = getConfidence(centerOutput);

        if (centerConfidence > 0.5) {
            teamElementLocation = BarcodeLocation.CENTER;
            return teamElementLocation;
        }

        if (leftConfidence > 0.2 || centerConfidence > 0.2) {
            Mat rawRightMat = mat.submat(new Rect(Constants.WEBCAM_SECTION_WIDTH, 0, Constants.WEBCAM_SECTION_WIDTH, Constants.WEBCAM_HEIGHT));
            Mat rightMat = new Mat();

            Imgproc.resize(rawRightMat, rightMat, new Size(224, 224));

            List<TensorImageClassifier.Recognition> rightOutput = imageClassifier.recognize(rightMat);

            float rightConfidence = getConfidence(rightOutput);

            if (rightConfidence > centerConfidence) {
                if (rightConfidence > leftConfidence) {
                    teamElementLocation = BarcodeLocation.RIGHT;
                } else {
                    teamElementLocation = BarcodeLocation.CENTER;
                }

            } else if (centerConfidence > leftConfidence) {
                teamElementLocation = BarcodeLocation.CENTER;
            } else {
                teamElementLocation = BarcodeLocation.LEFT;
            }
        } else {
            teamElementLocation = BarcodeLocation.RIGHT;
        }

        return teamElementLocation;
    }

    @Deprecated
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
        LEFT, CENTER, RIGHT
    }
}
