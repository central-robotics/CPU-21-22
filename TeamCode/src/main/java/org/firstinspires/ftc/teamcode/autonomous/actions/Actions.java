package org.firstinspires.ftc.teamcode.autonomous.actions;

import org.firstinspires.ftc.teamcode.autonomous.hardware.Hardware;
import org.firstinspires.ftc.teamcode.autonomous.localization.Localization;
import org.firstinspires.ftc.teamcode.autonomous.vision.Vuforia;

import java.util.ArrayList;
import java.util.List;

public class Actions {
    private final List<Action> actions = new ArrayList<>();
    private final Hardware hardware;
    private final Localization localization;
    private final Vuforia vuforia;

    public Actions(Hardware hardware, Localization localization, Vuforia vuforia)
    {
        this.hardware = hardware;
        this.localization = localization;
        this.vuforia = vuforia;
    }

    public void addTask(Action action)
    {
        actions.add(action);
    }

    // Executes the task at the waypoint with a given index
    public void executeTask(int index)
    {
        hardware.setAllMotorPowers(0);
        for (Action task : actions) {
            if (task.index == index) {
                task.execute(hardware, localization, vuforia);
                return;
            }
        }
    }

}
