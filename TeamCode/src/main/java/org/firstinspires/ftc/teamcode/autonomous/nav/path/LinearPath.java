package org.firstinspires.ftc.teamcode.autonomous.nav.path;

import org.firstinspires.ftc.teamcode.autonomous.localization.Position;
import org.firstinspires.ftc.teamcode.shared.control.PID;

public class LinearPath extends Path {
    public LinearPath(Position[] points, PID pid) {
        super(points, pid);
    }

    public LinearPath(Position[] points) {
        super(points);
    }
}
