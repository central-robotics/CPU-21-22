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
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;

import java.util.List;

public class ObjectDetector {
    private Vuforia vuforia;
    private OpenCvCamera camera;
    private ElementPipeline pipeline;
    private Hardware hardware;

    public ObjectDetector(Hardware hardware, Vuforia vuforia)
    {
        this.hardware = hardware;
        this.vuforia = vuforia;
        pipeline = new ElementPipeline();

        initializeObjectDetector();
    }

    private void initializeObjectDetector()
    {
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createVuforiaPassthrough(vuforia.vuforiaLocalizer, vuforia.parameters);
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
        return BarcodeLocation.values()[pipeline.getElementIndex()];
    }

    public enum BarcodeLocation
    {
        LEFT, CENTER, RIGHT;
    }
}
