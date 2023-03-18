package frc.robot.commands;

import static frc.robot.Constants.LedConstants.EPSILON;
import static frc.robot.Constants.LedConstants.MAX_ANGLE;

import java.util.function.DoubleSupplier;
import java.util.stream.IntStream;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.led_patches.SubStrip;
import frc.robot.utils.IndividualLed;

public class RollyPolly extends CommandBase {
    private final SubStrip strip;
    private final DoubleSupplier supplier;
    private final Color positive, negative;
    private Debouncer debouncer;
    private Debouncer fellOff;
    private boolean ended;

    public RollyPolly(SubStrip strip, DoubleSupplier angleSupplier, Color positive, Color negative) {
        this.strip = strip;
        supplier = angleSupplier;
        this.positive = positive;
        this.negative = negative;

        addRequirements(strip);
    }

    @Override
    public void initialize() {
        debouncer = new Debouncer(1.5, DebounceType.kRising);
        fellOff = new Debouncer(1.5, DebounceType.kRising);
        ended = false;
    }

    private void setRollingColor(double roll) {
        int index = (int) (((roll / MAX_ANGLE) + 1) * strip.size / 2);
        strip.setColor(
                IntStream.range(0, strip.size).mapToObj((i) -> new IndividualLed(i, i < index ? positive : negative))
                        .toArray(IndividualLed[]::new));
    }

    @Override
    public void execute() {
        double roll = supplier.getAsDouble();

        setRollingColor(roll);
        ended = debouncer.calculate(Math.abs(roll) < EPSILON);
        if (fellOff.calculate(Math.abs(roll) > MAX_ANGLE))
            new Rainbow(strip, 3).until(() -> Math.abs(supplier.getAsDouble()) < EPSILON).schedule();
    }

    @Override
    public boolean isFinished() {
        return ended;
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
