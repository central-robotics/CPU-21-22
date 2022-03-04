package org.firstinspires.ftc.teamcode.autonomous.nav.path;

import org.firstinspires.ftc.teamcode.autonomous.localization.Position;
import org.firstinspires.ftc.teamcode.shared.control.PID;

public abstract class Path {
    public Position[] points;
    public PID customPID;

    public Path(Position[] points) {
        this.points = points;
    }

    public Path(Position[] points, PID pid) {
        this.points = points;
        this.customPID = pid;
    }
}
