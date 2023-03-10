package frc.robot.commands;

import java.util.stream.IntStream;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.chassis.Chassis;
import frc.robot.subsystems.led_patches.SubStrip;
import frc.robot.utils.IndividualLed;

public class RollyPolly extends CommandBase {
    private final SubStrip strip;
    private final Chassis chassis;

    private static final Color LEVEL_COLOR = new Color(0, 255, 0);
    private static final double EPSILON = 1; // in degrees

    public RollyPolly(SubStrip strip, Chassis chassis) {
        this.strip = strip;
        this.chassis = chassis;

        addRequirements(strip);
    }

    @Override
    public void initialize() {
        setLevelColor();
    }

    private void setLevelColor() {
        strip.setColor(IntStream.range(0, strip.size).mapToObj((i) -> new IndividualLed(i, LEVEL_COLOR))
                .toArray(IndividualLed[]::new));
    }

    private void setRollingColor(double roll) {
        Color positive = new Color(255, 255, 0);
        Color negative = new Color(0, 0, 255);
        strip.setColor(
                IntStream.range(0, strip.size).mapToObj((i) -> new IndividualLed(i, roll > 0 ? positive : negative))
                        .toArray(IndividualLed[]::new));
    }

    @Override
    public void execute() {
        double roll = chassis.getRoll();

        if (Math.abs(roll) <= EPSILON)
            setLevelColor();
        else
            setRollingColor(roll);
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
