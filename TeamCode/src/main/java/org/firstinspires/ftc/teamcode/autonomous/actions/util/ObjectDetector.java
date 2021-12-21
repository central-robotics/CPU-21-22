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

import java.util.List;

public class ObjectDetector {
    private TFObjectDetector tfod;
    private Hardware hardware;
    private Vuforia vuforia;


    public ObjectDetector(Hardware hardware, Vuforia vuforia)
    {
        this.hardware = hardware;
        this.vuforia = vuforia;

        initializeObjectDetector();
    }

    private void initializeObjectDetector()
    {
        TFObjectDetector.Parameters params = new TFObjectDetector.Parameters();
        params.minResultConfidence = 0.7f;
        params.isModelTensorFlow2 = true;
        params.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(params, vuforia.vuforiaLocalizer);
    }

    //To allow the robot to figure out the position of any element without training an AI model, we will figure out which barcode we can't see.
    public Recognition getRecognition()
    {
        List<Recognition> recognitions = tfod.getUpdatedRecognitions();

        if (recognitions == null)
            return null;

        for (Recognition recognition : recognitions)
        {
            if (recognition.getLabel() == "CPU Custom Element" || recognition.getLabel() == "SIGMA Custom Element")
                return recognition;
        }

        return null;
    }

    public enum BarcodeLocation
    {
        LEFT, CENTER, RIGHT;
    }
}
