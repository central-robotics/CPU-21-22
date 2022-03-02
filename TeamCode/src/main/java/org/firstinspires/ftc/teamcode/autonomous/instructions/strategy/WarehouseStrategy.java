package org.firstinspires.ftc.teamcode.autonomous.instructions.strategy;

import org.firstinspires.ftc.teamcode.autonomous.Constants;
import org.firstinspires.ftc.teamcode.autonomous.actions.Actions;
import org.firstinspires.ftc.teamcode.autonomous.actions.PlaceCubeAction;
import org.firstinspires.ftc.teamcode.autonomous.actions.PlaceElementAction;
import org.firstinspires.ftc.teamcode.autonomous.actions.SpinIntakeAction;
import org.firstinspires.ftc.teamcode.autonomous.actions.util.ObjectDetector;
import org.firstinspires.ftc.teamcode.autonomous.hardware.Hardware;
import org.firstinspires.ftc.teamcode.autonomous.localization.Localization;
import org.firstinspires.ftc.teamcode.autonomous.localization.Position;
import org.firstinspires.ftc.teamcode.autonomous.vision.Vuforia;
import org.firstinspires.ftc.teamcode.autonomous.nav.Navigation;
import org.firstinspires.ftc.teamcode.autonomous.nav.path.LinearPath;
import org.firstinspires.ftc.teamcode.autonomous.nav.path.Path;
import org.firstinspires.ftc.teamcode.autonomous.nav.path.SplinePath;

import java.util.ArrayList;

public class WarehouseStrategy implements Strategy {
    @Override
    public Actions registerActions(Hardware hardware, Localization localization, Navigation navigation, Vuforia vuforia, ObjectDetector detector) {
        Actions actions = new Actions(hardware, localization, vuforia, detector);

        actions.addTask(new PlaceCubeAction(0, navigation));

        for (int i = 0; i < Constants.WAREHOUSE_ELEMENTS; i++)
        {
            actions.addTask(new SpinIntakeAction(1 + (i * 3)));
            actions.addTask(new PlaceElementAction(3 + (i*3)));
        }

        return actions;
    }

    @Override
    public ArrayList<Path> registerPath(double initialY, double initialTheta) {
        ArrayList<Path> path = new ArrayList<>();

        LinearPath p0 = new LinearPath(new Position[]{
                new Position(304.8, initialY, initialTheta)
        });

        path.add(p0);

        //PLACE CUBE ACTION

        for (int i = 0; i < Constants.WAREHOUSE_ELEMENTS; i++) {

            LinearPath p1 = new LinearPath(new Position[]{
                    new Position(243, 1900, Math.PI),
                    new Position(243, 2743, Math.PI)
            });

            path.add(p1);

            LinearPath p2 = new LinearPath(new Position[]{
                    new Position(243, 3139, Math.PI),
                    new Position(243, 3200, Math.PI),
            });

            path.add(p2);

            LinearPath p3 = new LinearPath(new Position[]{
                    new Position(243, 1828, Math.PI),
                    new Position(768, 1150, initialTheta)
            });

            path.add(p3);
        }

        SplinePath p4 = new SplinePath(new Position[]{
                new Position(495, 1500, 0),
                new Position(290, 1710, 0),
                new Position(204, 2090, 0),
                new Position(196, 2704, 0),
                new Position(414, 2980, 0),
                new Position(757,3067,0)
        });

        //path.add(p4);

        return path;
    }

    @Override
    public void setNav(Navigation nav) {

    }
}
