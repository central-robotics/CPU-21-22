package org.firstinspires.ftc.teamcode.autonomous.instructions.strategy;

import org.firstinspires.ftc.teamcode.autonomous.Constants;
import org.firstinspires.ftc.teamcode.autonomous.actions.Actions;
import org.firstinspires.ftc.teamcode.autonomous.actions.PlaceCubeAction;
import org.firstinspires.ftc.teamcode.autonomous.actions.SpinCarouselAction;
import org.firstinspires.ftc.teamcode.autonomous.actions.util.ObjectDetector;
import org.firstinspires.ftc.teamcode.autonomous.hardware.Hardware;
import org.firstinspires.ftc.teamcode.autonomous.localization.Localization;
import org.firstinspires.ftc.teamcode.autonomous.localization.Position;
import org.firstinspires.ftc.teamcode.autonomous.nav.Navigation;
import org.firstinspires.ftc.teamcode.autonomous.nav.path.LinearPath;
import org.firstinspires.ftc.teamcode.autonomous.nav.path.Path;
import org.firstinspires.ftc.teamcode.autonomous.vision.Vuforia;

import java.util.ArrayList;

public class CarouselStrategy implements Strategy {
    @Override
    public Actions registerActions(Hardware hardware, Localization localization, Navigation navigation, Vuforia vuforia, ObjectDetector detector) {
        Actions actions = new Actions(hardware, localization, vuforia, detector);

        actions.addTask(new PlaceCubeAction(0, navigation));
        actions.addTask(new SpinCarouselAction(1));

        return actions;
    }

    @Override
    public ArrayList<Path> registerPath(double initialY, double initialTheta) {
        ArrayList<Path> path = new ArrayList<>();

        LinearPath p0 = new LinearPath(new Position[]{
                new Position(304.8, initialY, initialTheta)
        });

        path.add(p0);

        //Rotate robot in the case that the carousel is on the opposite side of the robot.
        double rotation = Constants.IS_BLUE_TEAM ? initialTheta : 0;

        LinearPath p1 = new LinearPath(new Position[]{
                new Position(350.5, 350.5, rotation)
        });

        path.add(p1);

        LinearPath p2 = new LinearPath(new Position[]{
                new Position(914.4, 304.8, rotation),
        });

        path.add(p2);

        return path;
    }
}
