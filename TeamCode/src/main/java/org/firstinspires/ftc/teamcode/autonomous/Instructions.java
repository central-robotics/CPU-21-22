package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.autonomous.actions.Actions;
import org.firstinspires.ftc.teamcode.autonomous.actions.PlaceCubeAction;
import org.firstinspires.ftc.teamcode.autonomous.actions.SpinCarouselAction;
import org.firstinspires.ftc.teamcode.autonomous.hardware.Hardware;
import org.firstinspires.ftc.teamcode.autonomous.localization.Localization;
import org.firstinspires.ftc.teamcode.autonomous.localization.Position;
import org.firstinspires.ftc.teamcode.autonomous.waypoint.Navigation;
import org.firstinspires.ftc.teamcode.autonomous.waypoint.Waypoint;

/*
The Instructions class contains a map of waypoints and actions. It outlines all of the
tasks that the robot carries out during the autonomous phase. This class provides a method of
isolating these instructions to a separate class, rather than clogging up the AutonCore class.

This class also provides a method of disposing resources.
 */
public class Instructions {
    public Navigation navigation;
    public Actions actions;

    public Instructions(Hardware hardware, Localization localization, ElapsedTime runtime, Telemetry telemetry, LinearOpMode opMode)
    {
        registerActions(hardware, localization);
        registerNav(hardware, localization, runtime, actions, telemetry, opMode);
    }

    //Enter robot actions into this class.
    private void registerActions(Hardware hardware, Localization localization)
    {
        actions = new Actions(hardware, localization);
        //actions.addTask(new PlaceCubeAction(3, navigation));

        //actions.addTask(new SpinCarouselAction(1));
    }

    //Enter initial navigation waypoints here.
    private void registerNav(Hardware hardware, Localization localization, ElapsedTime runtime, Actions actions, Telemetry telemetry, LinearOpMode opMode)
    {
        navigation = new Navigation(hardware, localization, runtime, actions, telemetry, opMode);

        navigation.addWayPointToQueue(new Waypoint(new Position(0,0,0), new Position(0,-600,0)));
        navigation.addWayPointToQueue(new Waypoint(new Position(0,-600,0), new Position(600,-600,Math.PI /2 )));
        navigation.addWayPointToQueue(new Waypoint(new Position(600,-600,Math.PI /2), new Position(600,600, Math.PI)));
        navigation.addWayPointToQueue(new Waypoint(new Position(600,600,Math.PI), new Position(-600,600, Math.PI /2)));
        navigation.addWayPointToQueue(new Waypoint(new Position(-600,600,Math.PI / 2 ), new Position(-600,-600, 0)));
        navigation.addWayPointToQueue(new Waypoint(new Position(-600,-600,0), new Position(0,0, 0)));

    }

    public void runTasks()
    {
        navigation.executeTask();
    }
}
