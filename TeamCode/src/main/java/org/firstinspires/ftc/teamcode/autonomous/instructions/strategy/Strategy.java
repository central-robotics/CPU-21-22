package org.firstinspires.ftc.teamcode.autonomous.instructions.strategy;

import org.firstinspires.ftc.teamcode.autonomous.hardware.Hardware;
import org.firstinspires.ftc.teamcode.autonomous.localization.Localization;
import org.firstinspires.ftc.teamcode.autonomous.nav.Navigation;
import org.firstinspires.ftc.teamcode.autonomous.nav.path.Path;

import java.util.ArrayList;

public interface Strategy {
    void registerActions(Hardware hardware, Localization localization, Navigation navigation);
    ArrayList<Path> registerPath(double initialY, double initialTheta);
    void setNav(Navigation nav);
}
