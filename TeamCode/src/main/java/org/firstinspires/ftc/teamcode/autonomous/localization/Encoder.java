package org.firstinspires.ftc.teamcode.autonomous.localization;

import org.firstinspires.ftc.teamcode.autonomous.AutonCore;
import org.firstinspires.ftc.teamcode.autonomous.hardware.HardwareUtil;
import org.firstinspires.ftc.teamcode.autonomous.hardware.Motors;

import java.util.ArrayList;
import java.util.List;

public class Encoder {

    private ArrayList<Integer> lastPos; //last recorded
    private double distancePerTick = (2 * Math.PI * 48) / 537.6;

    public void initializeLocalizer()
    {
        lastPos = new ArrayList<>();
    }

    public ArrayList<Float> calculateDisplacements()
    {
        ArrayList<Float> motorDisplacements = new ArrayList<>();
        ArrayList<Integer> motorPositions = updateMotorPositions();

        int index = 0;

        for (Integer pos : motorPositions)
        {
            motorDisplacements.add((float) ((pos - lastPos.get(index)) * distancePerTick));
            index++;
        }

        return motorDisplacements;
    }

    private ArrayList<Integer> updateMotorPositions()
    {
        ArrayList<Integer> motorPositions = new ArrayList<>();

        motorPositions.add(Motors.leftFrontMotor.getCurrentPosition()); //0
        motorPositions.add(Motors.rightFrontMotor.getCurrentPosition()); //1
        motorPositions.add(Motors.rightBackMotor.getCurrentPosition()); //2
        motorPositions.add(Motors.leftBackMotor.getCurrentPosition()); //3

        lastPos = motorPositions;

        return motorPositions;
    }


}