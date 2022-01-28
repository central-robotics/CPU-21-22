package org.firstinspires.ftc.teamcode.autonomous.waypoint.path;

import org.firstinspires.ftc.teamcode.autonomous.actions.Actions;
import org.firstinspires.ftc.teamcode.autonomous.localization.Position;
import org.firstinspires.ftc.teamcode.autonomous.waypoint.path.util.ComputedSpline;

import java.util.ArrayList;

public class SplinePath extends Path{
    public ComputedSpline spline;
    public SplinePath(Position[] points) {
        super(points);
    }
}
