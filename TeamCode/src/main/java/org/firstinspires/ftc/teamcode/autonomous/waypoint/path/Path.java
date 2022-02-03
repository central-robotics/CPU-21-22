package org.firstinspires.ftc.teamcode.autonomous.waypoint.path;

import org.firstinspires.ftc.teamcode.autonomous.actions.Actions;
import org.firstinspires.ftc.teamcode.autonomous.localization.Position;

import java.util.ArrayList;

public abstract class Path {
    public Position[] points;

    public Path(Position[] points) {
        this.points = points;
    }
}
