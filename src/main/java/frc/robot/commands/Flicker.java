package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.led_patches.SubStrip;

public class Flicker extends CommandBase {
    private final SubStrip strip;
    private final Timer timer;
    private Color[] colors;

    public Flicker(SubStrip strip) {
        this.strip = strip;
        timer = new Timer();

        addRequirements(strip);
    }

    @Override
    public void initialize() {
        timer.restart();
        colors = strip.getColors();
    }

    @Override
    public void execute() {
        if (timer.get() < 0.3)
            strip.turnOff();
        else if (timer.get() < 0.6)
            strip.setColor(colors);
        else if (timer.get() < 0.9)
            strip.turnOff();
        else if (timer.get() < 1.2)
            strip.setColor(colors);
        else if (timer.get() < 1.5)
            strip.turnOff();
        else
            strip.setColor(colors);
    }

    @Override
    public boolean isFinished() {
        return timer.get() >= 1.8;
    }

    @Override
    public void end(boolean interrupted) {
        strip.turnOff();
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
