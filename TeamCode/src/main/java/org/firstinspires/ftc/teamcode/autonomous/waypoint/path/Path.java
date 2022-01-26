package org.firstinspires.ftc.teamcode.autonomous.waypoint.path;

import org.firstinspires.ftc.teamcode.autonomous.actions.Actions;
import org.firstinspires.ftc.teamcode.autonomous.localization.Position;

import java.util.ArrayList;

public abstract class Path {
    public ArrayList<Position> points;
    public Actions actions;

    public Path(ArrayList<Position> points, Actions actions) {
        this.points = points;
        this.actions = actions;
    }
}
