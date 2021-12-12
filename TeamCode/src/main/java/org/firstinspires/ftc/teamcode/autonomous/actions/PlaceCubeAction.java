package org.firstinspires.ftc.teamcode.autonomous.actions;

import org.firstinspires.ftc.teamcode.autonomous.actions.util.ObjectDetector;
import org.firstinspires.ftc.teamcode.autonomous.hardware.Hardware;
import org.firstinspires.ftc.teamcode.autonomous.localization.Localization;
import org.firstinspires.ftc.teamcode.autonomous.localization.Position;
import org.firstinspires.ftc.teamcode.autonomous.waypoint.Navigation;

public class PlaceCubeAction extends Action {
    private Navigation navigation;
    public PlaceCubeAction(int index, Navigation navigation) {
        super(index);
        this.navigation = navigation;
    }

    @Override
    public void execute(Hardware hardware, Localization localization) {
        ObjectDetector objectDetector = new ObjectDetector(hardware);

        switch (objectDetector.getRecognition())
        {
            case LEFT:
                navigation.moveToTarget(new Position(0,0,0));
                //pick cube up
                break;
            case CENTER:
                navigation.moveToTarget(new Position(0,0,0));
                //pick cube up
                break;
            case RIGHT:
                navigation.moveToTarget(new Position(0,0,0));
                //pick cube up
                break;
        }
    }
}
