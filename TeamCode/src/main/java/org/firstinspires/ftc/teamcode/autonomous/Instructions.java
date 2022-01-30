package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.autonomous.actions.Actions;
import org.firstinspires.ftc.teamcode.autonomous.actions.PlaceCubeAction;
import org.firstinspires.ftc.teamcode.autonomous.actions.PlaceElementAction;
import org.firstinspires.ftc.teamcode.autonomous.actions.SpinCarouselAction;
import org.firstinspires.ftc.teamcode.autonomous.actions.SpinIntakeAction;
import org.firstinspires.ftc.teamcode.autonomous.hardware.Hardware;
import org.firstinspires.ftc.teamcode.autonomous.localization.Localization;
import org.firstinspires.ftc.teamcode.autonomous.localization.Position;
import org.firstinspires.ftc.teamcode.autonomous.vision.Vuforia;
import org.firstinspires.ftc.teamcode.autonomous.waypoint.Navigation;
import org.firstinspires.ftc.teamcode.autonomous.waypoint.path.LinearPath;

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

        if (!Constants.IS_BLUE_TEAM) //RED ALLIANCE
        {
            if (Constants.IS_LEFT_OPMODE) //CAROUSEL SIDE
            {
                actions.addTask(new PlaceCubeAction(0, navigation));
                actions.addTask(new SpinCarouselAction(1));
            } else
            {
                actions.addTask(new PlaceCubeAction(0, navigation));

                for (int i = 0; i < Constants.WAREHOUSE_ELEMENTS; i++)
                {
                    actions.addTask(new SpinIntakeAction(1 + (i * 3)));
                    actions.addTask(new SpinIntakeAction(2 + (i * 3)));
                    actions.addTask(new PlaceElementAction(3+ (i*3)));
                }
            }
        } else //BLUE ALLIANCE
        {
            if (Constants.IS_LEFT_OPMODE) //WAREHOUSE SIDE
            {
                actions.addTask(new PlaceCubeAction(0, navigation));

                for (int i = 0; i < Constants.WAREHOUSE_ELEMENTS; i++)
                {
                    actions.addTask(new SpinIntakeAction(1 + (i * 3)));
                    actions.addTask(new SpinIntakeAction(2 + (i * 3)));
                    actions.addTask(new PlaceElementAction(3+ (i*3)));
                }
            } else
            {
                actions.addTask(new PlaceCubeAction(0, navigation));
                actions.addTask(new SpinCarouselAction(1));
            }
        }
    }

    //Enter initial navigation waypoints here.
    private void registerNav(Hardware hardware, Localization localization, ElapsedTime runtime, Actions actions, Telemetry telemetry, LinearOpMode opMode, double initialX, double initialY, double initialTheta)
    {
        navigation = new Navigation(hardware, localization, runtime, actions, telemetry, opMode);

        double rotation;

        // If we are on the red side, carousel is on the opposite side of robot. We need to rotate the robot the robot if we are spinning the carousel.

        if (!Constants.IS_BLUE_TEAM) //RED ALLIANCE
        {
            if (Constants.IS_LEFT_OPMODE) //CAROUSEL SIDE
            {
                LinearPath p0 = new LinearPath(new Position[]{
                        new Position(304.8, initialY, initialTheta)
                });

                navigation.addPathToPipeline(p0);

                //PLACE CUBE

                LinearPath p1 = new LinearPath(new Position[]{
                        new Position(350.5, 350.5, 0)
                });

                navigation.addPathToPipeline(p1);

                //CAROUSEL

                LinearPath p2 = new LinearPath(new Position[]{
                        new Position(914.4, 304.8, 0),
                });

                navigation.addPathToPipeline(p2);

                //PARK
            } else
            {
                LinearPath p0 = new LinearPath(new Position[]{
                        new Position(304.8, initialY, initialTheta)
                });

                navigation.addPathToPipeline(p0);

                //PLACE CUBE ACTION

                for (int i = 0; i < Constants.WAREHOUSE_ELEMENTS; i++) {

                    LinearPath p1 = new LinearPath(new Position[]{
                            new Position(243, 1828, 0),
                            new Position(243, 2743, 0)
                    });

                    navigation.addPathToPipeline(p1);

                    //START SPINNING INTAKE

                    LinearPath p2 = new LinearPath(new Position[]{
                            new Position(243, 3139, 0),
                            new Position(243, 3200, 0)
                    });

                    navigation.addPathToPipeline(p2);

                    //GO BACK TO HUB

                    LinearPath p3 = new LinearPath(new Position[]{
                            new Position(243, 1828, 0),
                            new Position(768, 1524, initialTheta)
                    });

                    navigation.addPathToPipeline(p3);

                    //PLACE CUBE
                }

                LinearPath p4 = new LinearPath(new Position[]{
                        new Position(243, 1828, 0),
                        new Position(243, 3000, 0),
                        new Position(700, 3000, 0)
                });

                navigation.addPathToPipeline(p4);

                //PARK AND GET OUT OF WAY
            }
        } else //BLUE ALLIANCE
        {
            if (Constants.IS_LEFT_OPMODE) //WAREHOUSE SIDE
            {
                LinearPath p0 = new LinearPath(new Position[]{
                        new Position(304.8, initialY, initialTheta)
                });

                navigation.addPathToPipeline(p0);

                //PLACE CUBE ACTION

                for (int i = 0; i < Constants.WAREHOUSE_ELEMENTS; i++) {

                    LinearPath p1 = new LinearPath(new Position[]{
                            new Position(243, 1828, 0),
                            new Position(243, 2743, 0)
                    });

                    navigation.addPathToPipeline(p1);

                    //START SPINNING INTAKE

                    LinearPath p2 = new LinearPath(new Position[]{
                            new Position(243, 3139, 0),
                            new Position(243, 3200, 0)
                    });

                    navigation.addPathToPipeline(p2);

                    //GO BACK TO HUB

                    LinearPath p3 = new LinearPath(new Position[]{
                            new Position(243, 1828, 0),
                            new Position(768, 1524, initialTheta)
                    });

                    navigation.addPathToPipeline(p3);

                    //PLACE CUBE
                }

                LinearPath p4 = new LinearPath(new Position[]{
                        new Position(243, 1828, 0),
                        new Position(243, 3000, 0),
                        new Position(700, 3000, 0)
                });

                navigation.addPathToPipeline(p4);

                //PARK AND GET OUT OF WAY
            } else
            {
                LinearPath p0 = new LinearPath(new Position[]{
                        new Position(304.8, initialY, initialTheta)
                });

                navigation.addPathToPipeline(p0);

                //PLACE CUBE

                LinearPath p1 = new LinearPath(new Position[]{
                        new Position(350.5, 350.5, 0)
                });

                navigation.addPathToPipeline(p1);

                //CAROUSEL

                LinearPath p2 = new LinearPath(new Position[]{
                        new Position(914.4, 304.8, 0),
                });

                navigation.addPathToPipeline(p2);

                //PARK
            }
        }
    }

    public void runTasks()
    {
        navigation.executeTask();
    }
}
