package org.firstinspires.ftc.teamcode.autonomous.localization;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.autonomous.Constants;
import org.firstinspires.ftc.teamcode.autonomous.hardware.Hardware;

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

    public Position getRobotPosition(Position previousPosition)
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
        double dispAvg;

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

        //Compute displacement in robot reference frame.
        deltaX = (robotLfDisp + robotRfDisp - robotRbDisp - robotLbDisp) / (2 * Math.sqrt(2));
        deltaY = (robotLfDisp - robotRfDisp - robotRbDisp + robotLbDisp) / (2 * Math.sqrt(2));

        //Robot theta
        double theta;

        //Compute robot theta
        theta = _hardware.gyro.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS).thirdAngle;

        //Displacement in field reference frame.
        double deltaXf;
        double deltaYf;

        //Compute displacement in field reference frame.
        deltaXf = deltaX * Math.cos(theta) - deltaY * Math.sin(theta);
        deltaYf = deltaY * Math.cos(theta) + deltaX * Math.sin(theta);

        Position robotPosition = new Position();
        robotPosition.x = previousPosition.x + deltaXf;
        robotPosition.y = previousPosition.y + deltaYf;
        robotPosition.t = theta;

        return robotPosition;
    }

    public Velocity getRobotVelocity(Position previousPosition, Position currentPosition, double previousTime, double currentTime)
    {
        Velocity robotVelocity = new Velocity();

        robotVelocity.dx = (currentPosition.x - previousPosition.x) / (currentTime - previousTime);
        robotVelocity.dy = (currentPosition.y - previousPosition.y) / (currentTime - previousTime);
        robotVelocity.dt = (currentPosition.t - previousPosition.t) / (currentTime - previousTime);

        return robotVelocity;
    }
}