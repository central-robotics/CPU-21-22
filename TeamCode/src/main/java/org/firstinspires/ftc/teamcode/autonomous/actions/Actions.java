package org.firstinspires.ftc.teamcode.autonomous.actions;

import org.firstinspires.ftc.teamcode.autonomous.hardware.Hardware;
import org.firstinspires.ftc.teamcode.autonomous.localization.Localization;

import java.util.ArrayList;
import java.util.List;

public class Actions {
    private final Hardware _hardware;
    private List<ActionTask> _actions;
    private Localization _localization;

    public Actions(Hardware hardware, Localization localization)
    {
        _localization = localization;
        _hardware = hardware;
        _actions = new ArrayList<>();
    }

    public void addTask(Action action, int index)
    {
        _actions.add(new ActionTask(action, index));
    }

    //Checks to see if there exists a task on a certain waypoint, if yes, execute that task. Preserves the privacy of the _actions list.
    public void executeTask(int index)
    {
        for (ActionTask task : _actions)
            if (task.index == index)
            {
                switch (task.action)
                {
                    case SPIN_CAROUSEL:
                        spinCarousel();
                        return;
                    case PLACE_CUBE:
                        placeCube();
                        return;
                }
            }
    }

    private void spinCarousel()
    {
        _hardware.carouselMotor.setTargetPosition(_hardware.carouselMotor.getCurrentPosition() + 1000); //Spin 1000 ticks. We don't care about the exact position, so we'll just add it to our current tick count.
        _hardware.carouselMotor.setPower(0.4);
    }

    private void placeCube()
    {

    }

    public enum Action
    {
        SPIN_CAROUSEL, PLACE_CUBE;
    }

    private class ActionTask
    {
        public ActionTask(Action action, int index)
        {
            this.action = action;
            this.index = index;
        }

        public Action action; //Specifies the action we want to carry out.
        public Integer index; //Specifies which waypoint we want to execute task on.
    }
}
