package org.firstinspires.ftc.teamcode.autonomous.nav.path;

import org.firstinspires.ftc.teamcode.autonomous.localization.Position;

public abstract class Path {
    public Position[] points;

    public Path(Position[] points) {
        this.points = points;
    }
}
