package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.autonomous.actions.Actions;
import org.firstinspires.ftc.teamcode.autonomous.actions.PlaceCubeAction;
import org.firstinspires.ftc.teamcode.autonomous.actions.SpinCarouselAction;
import org.firstinspires.ftc.teamcode.autonomous.actions.SpeedrunAction;
import org.firstinspires.ftc.teamcode.autonomous.hardware.Hardware;
import org.firstinspires.ftc.teamcode.autonomous.localization.Localization;
import org.firstinspires.ftc.teamcode.autonomous.localization.Position;
import org.firstinspires.ftc.teamcode.autonomous.vision.Vuforia;
import org.firstinspires.ftc.teamcode.autonomous.waypoint.Navigation;
import org.firstinspires.ftc.teamcode.autonomous.waypoint.Waypoint;
import org.firstinspires.ftc.teamcode.autonomous.waypoint.path.Path;
import org.firstinspires.ftc.teamcode.autonomous.waypoint.path.SplinePath;
import org.firstinspires.ftc.teamcode.autonomous.waypoint.path.util.SplineHelper;

/*
The Instructions class contains a map of waypoints and actions. It outlines all of the
tasks that the robot carries out during the autonomous phase. This class provides a method of
isolating these instructions to a separate class, rather than clogging up the AutonCore class.

This class also provides a method of disposing resources.
 */
public class Instructions {
    public Navigation navigation;
    public Actions actions;

    public Instructions(Hardware hardware, Localization localization, ElapsedTime runtime, Telemetry telemetry, LinearOpMode opMode, Vuforia vuforia, double initialX, double initialY, double initialTheta)
    {
        registerActions(hardware, localization, vuforia);
        registerNav(hardware, localization, runtime, actions, telemetry, opMode, initialX, initialY, initialTheta);
    }

    //Enter robot actions into this class.
    private void registerActions(Hardware hardware, Localization localization, Vuforia vuforia)
    {
        actions = new Actions(hardware, localization, vuforia);
        if (!Constants.IS_LEFT_OPMODE) {
             actions.addTask(new SpinCarouselAction(0));
        }
        else{
            actions.addTask(new SpeedrunAction(0));
        }
        //actions.addTask(new PlaceCubeAction(3, navigation));

        //actions.addTask(new SpinCarouselAction(1));
    }

    //Enter initial navigation waypoints here.
    private void registerNav(Hardware hardware, Localization localization, ElapsedTime runtime, Actions actions, Telemetry telemetry, LinearOpMode opMode, double initialX, double initialY, double initialTheta)
    {
        navigation = new Navigation(hardware, localization, runtime, actions, telemetry, opMode);

        double rotation;
        if (!Constants.IS_LEFT_OPMODE) {
            if (!Constants.IS_BLUE_TEAM)
                rotation = initialTheta;
            else
                rotation = 0;
        }
        else{
            //Some oth
        }

        SplinePath path = new SplinePath(new Position[]{
                new Position(0, 0, 0),
                new Position(330, 230, 0),
                new Position(550, 600, 0),
        });

        navigation.addPathToPipeline(path);

        try {
            navigation.calculateSplines();
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    public void runTasks()
    {
        navigation.executeTask();
    }
}
