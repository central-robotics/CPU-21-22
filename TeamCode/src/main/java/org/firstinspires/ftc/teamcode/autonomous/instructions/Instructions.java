package org.firstinspires.ftc.teamcode.autonomous.instructions;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.autonomous.Constants;
//import org.firstinspires.ftc.teamcode.autonomous.actions.Actions;
//import org.firstinspires.ftc.teamcode.autonomous.actions.util.ObjectDetector;
import org.firstinspires.ftc.teamcode.autonomous.hardware.Hardware;
import org.firstinspires.ftc.teamcode.autonomous.instructions.strategy.CarouselStrategy;
import org.firstinspires.ftc.teamcode.autonomous.instructions.strategy.Strategy;
import org.firstinspires.ftc.teamcode.autonomous.instructions.strategy.WarehouseStrategy;
import org.firstinspires.ftc.teamcode.autonomous.localization.Localization;
//import org.firstinspires.ftc.teamcode.autonomous.vision.Vuforia;
import org.firstinspires.ftc.teamcode.autonomous.nav.Navigation;
import org.firstinspires.ftc.teamcode.autonomous.nav.path.Path;

/*
The Instructions class contains a map of waypoints and actions. It outlines all of the
tasks that the robot carries out during the autonomous phase. This class provides a method of
isolating these instructions to a separate class, rather than clogging up the AutonCore class.

This class also provides a method of disposing resources.
 */
public class Instructions {
    public static Navigation navigation;
//    public Actions actions;

    public Instructions(Hardware hardware, Localization localization, ElapsedTime runtime, Telemetry telemetry, LinearOpMode opMode, Object vuforia, Object detector, double initialX, double initialY, double initialTheta)
    {
        Strategy strategy = getStrategy();
//        registerActions(hardware, localization, null, null, strategy);
        registerNav(hardware, localization, runtime, null, telemetry, opMode, initialX, initialY, initialTheta, strategy);
    }

    private void registerActions(Hardware hardware, Localization localization, Object vuforia, Object detector, Strategy strategy)
    {
//        actions = strategy.registerActions(hardware, localization, navigation, vuforia, detector);
    }

    private void registerNav(Hardware hardware, Localization localization, ElapsedTime runtime, Object actions, Telemetry telemetry, LinearOpMode opMode, double initialX, double initialY, double initialTheta, Strategy strategy)
    {
        navigation = new Navigation(hardware, localization, runtime, null, telemetry, opMode);

        for (Path p : strategy.registerPath(initialY, initialTheta))
            navigation.addPathToPipeline(p);
    }

    private Strategy getStrategy()
    {
        if (Constants.IS_BLUE_TEAM) {
            if (Constants.IS_LEFT_OPMODE)
                return new WarehouseStrategy();
            else
                return new CarouselStrategy();
        } else
            if (Constants.IS_LEFT_OPMODE)
                return new CarouselStrategy();
            else
                return new WarehouseStrategy();
    }

    public void runTasks()
    {
        navigation.executeTask();
    }
}
