package org.firstinspires.ftc.teamcode.autonomous.localization;

import org.firstinspires.ftc.teamcode.autonomous.Constants;
import org.firstinspires.ftc.teamcode.autonomous.hardware.Hardware;
import org.firstinspires.ftc.teamcode.autonomous.hardware.Motors;

import java.util.ArrayList;

public class Encoder {
    private Hardware _hardware; //Contains robot hardware for measuring robot position using motor encoders.

    //private final double distancePerTick = (2 * Math.PI * 48) / 537.6;
    private final double distancePerTick = 537.6;

    //Position of encoder on each motor respectively. We need to store this so that we can subtract these values from current position to get displacement.
    private int lastLfPos, lastRfPos, lastRbPos, lastLbPos;

    public Encoder(Hardware hardware)
    {
        _hardware = hardware;
    }

    public Position getRobotPosition()
    {
        //Encoder values. These are in ticks. We will later convert this to a usable distance.
        int lfPos, rfPos, rbPos, lbPos;

        //Record encoder values.
        lfPos = _hardware.leftFrontMotor.getCurrentPosition();
        rfPos = _hardware.rightFrontMotor.getCurrentPosition();
        rbPos= _hardware.rightBackMotor.getCurrentPosition();
        lbPos = _hardware.leftBackMotor.getCurrentPosition();

        //Displacement values
        double lfDisp, rfDisp, rbDisp, lbDisp;

        //Calculate displacement values
        lfDisp = (lfPos - lastLfPos) * Constants.DISTANCE_PER_TICK;
        rfDisp = (rfPos - lastRfPos) * Constants.DISTANCE_PER_TICK;
        rbDisp = (rbPos - lastRbPos) * Constants.DISTANCE_PER_TICK;
        lbDisp = (lbPos - lastLbPos) * Constants.DISTANCE_PER_TICK;

        //Store encoder values so we can use them in calculating displacement.
        lastLfPos = lfPos;
        lastRfPos = rfPos;
        lastRbPos = rbPos;
        lastLbPos = lbPos;

        //Average of displacement values
        double dispAvg = 0.0;

        //Calculate avg.
        dispAvg = (lfDisp + rfDisp + rbDisp + lbDisp) / 4;

        //Robot displacement
        double robotLfDisp, robotRfDisp, robotRbDisp, robotLbDisp;

        //Calculate robot displacement
        robotLfDisp = lfDisp - dispAvg;
        robotRfDisp = rfDisp - dispAvg;
        robotRbDisp = rbDisp - dispAvg;
        robotLbDisp = lbDisp - dispAvg;

        //Holonomic displacement in robot reference frame.
        double deltaX, deltaY;
    }

    public ArrayList<Double> calculateDisplacements() {
        ArrayList<Double> motorDisplacements = new ArrayList<>();
        ArrayList<Integer> motorPositions = updateMotorPositions();

        int index = 0;

        for (Integer pos : motorPositions) {
            motorDisplacements.add ((pos - lastPos.get(index)) * distancePerTick);
            index++;
        }

        return motorDisplacements;
    }

    private ArrayList<Integer> updateMotorPositions() {
        ArrayList<Integer> motorPositions = new ArrayList<>();

        motorPositions.add(Motors.leftFrontMotor.getCurrentPosition()); //0
        motorPositions.add(Motors.rightFrontMotor.getCurrentPosition()); //1
        motorPositions.add(Motors.rightBackMotor.getCurrentPosition()); //2
        motorPositions.add(Motors.leftBackMotor.getCurrentPosition()); //3

        lastPos = motorPositions;

        return motorPositions;
    }


}