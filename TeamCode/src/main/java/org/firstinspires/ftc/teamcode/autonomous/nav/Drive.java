package org.firstinspires.ftc.teamcode.autonomous.nav;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.autonomous.Constants;
import org.firstinspires.ftc.teamcode.autonomous.control.PID;
import org.firstinspires.ftc.teamcode.autonomous.hardware.Hardware;
import org.firstinspires.ftc.teamcode.autonomous.localization.Localization;
import org.firstinspires.ftc.teamcode.autonomous.localization.Position;
import org.firstinspires.ftc.teamcode.autonomous.localization.Velocity;
import org.firstinspires.ftc.teamcode.autonomous.nav.path.LinearPath;
import org.firstinspires.ftc.teamcode.autonomous.nav.path.Path;
import org.firstinspires.ftc.teamcode.autonomous.nav.path.util.ParametricSpline;
import org.firstinspires.ftc.teamcode.autonomous.nav.path.util.SplineHelper;

public class Drive {
    private final Localization localization;
    private final Hardware hardware;
    private PID controller;
    private PID thetaSplineController;
    private PID thetaLinearController;
    private Position position;
    private Position prevPosition;
    private final int splinePoint = 0;
    private final ElapsedTime runtime;
    private final LinearOpMode opMode;
    private float dist;

    public Drive(Localization localization, Hardware hardware, ElapsedTime runtime, LinearOpMode opMode) {
        this.localization = localization;
        this.hardware = hardware;
        this.runtime = runtime;
        this.opMode = opMode;

        initializeDrive();
    }
    private void initializeDrive()
    {
        PIDCoefficients coefficients = new PIDCoefficients(0.0115, 0.00006, 0);
        PIDCoefficients thetaLinCoefficients = new PIDCoefficients(0.22, 0.0004, 0);
        PIDCoefficients thetaSplCoefficients = new PIDCoefficients(0.10, 0.0004, 0);

        controller = new PID(coefficients);
        thetaLinearController = new PID(thetaLinCoefficients);
        thetaSplineController = new PID(thetaSplCoefficients);


        position = new Position();

    }

    public void driveAlongPath(Path path, Telemetry telem) {

        if (path.getClass() == LinearPath.class) {
            for (int i = 0; i < path.points.length; i++) {
                Position target = path.points[i];
                calculateMotorPowers(target, path, null, telem);
            }
        } else
        {
            dist = 0;

            SplineHelper splineHelper = new SplineHelper();

            position = localization.getRobotPosition();
            localization.increment(position);
            prevPosition = position;

            double[] x, y;
            x = new double[path.points.length + 1];
            y = new double[path.points.length + 1];

            //We want the first point of the spline to be where the robot is.
            //This is because the spline controller mathematically assumes that the first control point has been reached.
            x[0] = position.x;
            y[0] = position.y;

            for (int i = 0; i < path.points.length; i++)
            {
                x[i+1] = path.points[i].x;
                y[i+1] = path.points[i].y;
            }

            Position[] newPoints = new Position[x.length];

            for (int i = 0; i < x.length; i++)
            {
                newPoints[i] = new Position(x[i], y[i], 0);
            }

            path.points = newPoints;

            ParametricSpline spline = splineHelper.computeSpline(x, y);

            Position target = path.points[path.points.length - 1];

            calculateMotorPowers(target, path, spline, telem);
        }
    }

    public void setSplinePowers(Path path, ParametricSpline spline, Telemetry telem) {
        if (opMode.isStopRequested())
            opMode.stop();

        position = localization.getRobotPosition();
        localization.increment(position);

        dist += Math.sqrt(Math.pow(position.x - prevPosition.x, 2) + Math.pow(position.y - prevPosition.y, 2));

        prevPosition = position;
        double t = dist;

        if (t >= spline.splineDistance - 5)
            return;


        double orientation;

        if (spline.xSpline.derivative().value(t) > 0)
            orientation = Math.atan(spline.getDerivative(t)) - (Math.PI / 4) + (Math.PI / 2);
        else
            orientation = Math.atan(spline.getDerivative(t)) + Math.PI - Math.PI / 4 + (Math.PI / 2);

        if (!Constants.IS_BLUE_TEAM)
            orientation += Math.PI;

        double speed = Math.abs(0.6 - (spline.getCurvature(t) * 100));

        if (speed < 0.2)
            speed = 0.2;

        telem.addData("TARGET POS X", Math.abs(path.points[path.points.length - 1].x));
        telem.addData("TARGET POS Y", Math.abs(path.points[path.points.length - 1].y));
        telem.addData("TARGET POS T", Math.abs(path.points[path.points.length - 1].t));
        telem.addData("DESIRED POS X", spline.xSpline.value(t));
        telem.addData("DESIRED POS Y", spline.ySpline.value(t));
        telem.addData("T", dist);
        telem.addData("CURRENT POS X", position.x);
        telem.addData("CURRENT POS Y", position.y);
        telem.addData("CURRENT POS T", position.t);
        telem.addData("SPEED", speed);
        telem.addData("ORIENTATION", orientation + Math.PI / 4);
        telem.addData("D", spline.getDerivative(t));
        telem.update();

        double negativePower = 0;
        double positivePower = 0;

        if (Double.isNaN(speed))
            speed = 0.6;

        negativePower = speed * Math.sin(orientation);

        if (orientation == 0)
            positivePower = negativePower;
        else
            positivePower = speed * Math.cos(orientation);

        hardware.setMotorValues(positivePower, negativePower);
    }

    public void setLinearPowers(Position target, double thetaError, boolean isCounterClockwise, Telemetry telem)
    {
        if (opMode.isStopRequested())
            return;

        position = localization.getRobotPosition();
        localization.increment(position);

        telem.addData("TARGET POS X", target.x);
        telem.addData("TARGET POS Y", target.y);
        telem.addData("TARGET POS T", target.t);
        telem.addData("CURRENT POS X", position.x);
        telem.addData("CURRENT POS Y", position.y);
        telem.addData("CURRENT POS T", position.t);
        telem.update();

        Velocity velocity = localization.getRobotVelocity(runtime);
        double orientation, magnitude, negOutput, posOutput;

        if (target.x - position.x > 0)
            orientation = Math.atan(controller.getSlope(target, position)) - Math.PI / 4 - position.t;
        else
            orientation = Math.atan(controller.getSlope(target, position)) + Math.PI - Math.PI / 4 - position.t;

        double error = Math.sqrt(Math.pow(target.y - position.y, 2) + Math.pow(target.x - position.x, 2));
        double speed = Math.sqrt(Math.pow(velocity.dy, 2) + Math.pow(velocity.dx, 2));

        magnitude = controller.getOutput(error, speed);

        if (error < 3) { // Make magnitude 0 if error is too low to matter
            magnitude = 0;
        }

        negOutput = magnitude * Math.sin(orientation);
        if (orientation == 0)
            posOutput = negOutput;
        else
            posOutput = magnitude * Math.cos(orientation);

        double thetaOutput = thetaLinearController.getOutput(Math.abs(thetaError), 0);

        if (Math.abs(thetaError) < Constants.THETA_TOLERANCE) { // Set thetaOutput to 0 if thetaError is negligible
            thetaOutput = 0;
        }

        hardware.setMotorValuesWithRotation(0.1 * posOutput, 0.1 * negOutput, (isCounterClockwise ? -1 : 1) * thetaOutput);
    }

    private void calculateMotorPowers(Position target, Path path, ParametricSpline spline , Telemetry telem)
    {
        boolean thetaFinished = false;
        dist = 0;

        while (((Math.abs(target.x - position.x) > 8) || (Math.abs(target.y - position.y) > 8) || !thetaFinished) && !opMode.isStopRequested()) {
            position = localization.getRobotPosition();
            localization.increment(position);
            thetaFinished = false;

            double thetaError = target.t - position.t;
            boolean isCounterClockwise = (thetaError) > 0 && (thetaError < Math.PI);

            if ((thetaError) < 0 && (thetaError < -Math.PI)) {
                isCounterClockwise = true;
                thetaError = target.t - position.t + (2 * Math.PI);
            }

            if (thetaError < Constants.THETA_TOLERANCE) {
                thetaFinished = true;
            }

            if (path.getClass() == LinearPath.class)
                setLinearPowers(target, thetaError, isCounterClockwise, telem);
            else
            {
                if (dist >= spline.splineDistance - 5)
                    break;
                position = localization.getRobotPosition();
                localization.increment(position);

                if (!thetaFinished)
                    turnMovement(thetaError, isCounterClockwise);
                else
                    setSplinePowers(path, spline, telem);
            }
        }

        controller.resetSum();
        thetaLinearController.resetSum();
        thetaSplineController.resetSum();
    }

    private void turnMovement(double thetaError, boolean isCounterClockwise)
    {
        double thetaOutput = thetaSplineController.getOutput(thetaError, 0);

        if (Math.abs(thetaError) < Constants.THETA_TOLERANCE) { // Set thetaOutput to 0 if thetaError is negligible
        thetaOutput = 0;
    }

        hardware.setAllMotorPowers((isCounterClockwise ? 1 : -1) * thetaOutput);
    }

    @Deprecated
    public void driveToTarget(Position destination)
    {
        boolean thetaFinished = false;
        dist = 0;


        //Assume that starting position has been reached. Drive to target specified by waypoint.
        while(((Math.abs(destination.x - position.x) > 8) || (Math.abs(destination.y - position.y) > 8) || !thetaFinished) && !opMode.isStopRequested()) {
            position = localization.getRobotPosition();
            localization.increment(position);
            thetaFinished = false;

            double thetaError = destination.t - position.t;
            boolean isCounterClockwise = (thetaError) > 0 && (thetaError < Math.PI);

            if ((thetaError) < 0 && (thetaError < -Math.PI)) {
                isCounterClockwise = true;
                thetaError = destination.t - position.t + (2 * Math.PI);
            }

            if (thetaError < Constants.THETA_TOLERANCE){
                thetaFinished = true;
            }

            //setMotorPowers(destination, thetaError, isCounterClockwise);
        }

        controller.resetSum();
    }
}
