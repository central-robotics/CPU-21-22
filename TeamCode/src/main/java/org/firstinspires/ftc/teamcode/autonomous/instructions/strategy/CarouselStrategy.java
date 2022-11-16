package org.firstinspires.ftc.teamcode.autonomous.instructions.strategy;

import org.firstinspires.ftc.teamcode.autonomous.Constants;
import org.firstinspires.ftc.teamcode.autonomous.hardware.Hardware;
import org.firstinspires.ftc.teamcode.autonomous.localization.Localization;
import org.firstinspires.ftc.teamcode.autonomous.localization.Position;
import org.firstinspires.ftc.teamcode.autonomous.nav.Navigation;
import org.firstinspires.ftc.teamcode.autonomous.nav.path.LinearPath;
import org.firstinspires.ftc.teamcode.autonomous.nav.path.Path;
import org.firstinspires.ftc.teamcode.autonomous.nav.path.SplinePath;

import java.util.ArrayList;

public class CarouselStrategy implements Strategy {

    @Override
    public void registerActions(Hardware hardware, Localization localization, Navigation navigation) {
    }

    @Override
    public ArrayList<Path> registerPath(double initialY, double initialTheta) {
        ArrayList<Path> path = new ArrayList<>();
        path.add(new SplinePath(new Position[]{new Position(0,0,0), new Position(0, 1000, 0), new Position(1000, 1000, 0)}));
        return path;
//        LinearPath p0 = new LinearPath(new Position[]{
//                new Position(304.8, initialY, initialTheta)
//        });
//
//        path.add(p0);
//
//        Rotate robot in the case that the carousel is on the opposite side of the robot.
//        double rotation = Constants.IS_BLUE_TEAM ? initialTheta - ( 5  *Math.PI) / 4 : (3 * Math.PI ) / 4;
//
//        LinearPath p1 = new LinearPath(new Position[]{
//                new Position(380, 500, rotation),
//                new Position(350, 200, rotation)
//
//        });
//
//        path.add(p1);
//
//        Position pos;
//
//        if (Constants.IS_BLUE_TEAM){
//            pos = new Position(970, 200, initialTheta);
//        }
//        else{
//            pos = new Position(970, 40, initialTheta);
//        }
//
//        LinearPath p2 = new LinearPath(new Position[]{
//                pos
//        });
//
//        path.add(p2);
//
//        return path;
    }

    @Override
    public void setNav(Navigation nav) {

    }
}
