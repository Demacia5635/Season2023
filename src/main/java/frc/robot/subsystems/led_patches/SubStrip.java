package frc.robot.subsystems.led_patches;

import java.util.Arrays;
import java.util.stream.IntStream;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.LedsManager;
import frc.robot.utils.IndividualLed;

public class SubStrip extends SubsystemBase {
    public final int size;
    private final int offset;

    public SubStrip(int offset, int size) {
        this.offset = offset;
        this.size = size;
    }

    public void setColor(IndividualLed... leds) {
        LedsManager.getInstance().update(Arrays.stream(leds)
                .map((led) -> new IndividualLed(offset + led.index, led.color)).toArray(IndividualLed[]::new));
    }

    public void setColor(Color color) {
        setColor(IntStream.range(0, size).mapToObj((i) -> new IndividualLed(i, color))
                .toArray(IndividualLed[]::new));
    }

    public void turnOff() {
        setColor(IntStream.range(0, size).mapToObj((i) -> new IndividualLed(i, Color.kBlack))
                .toArray(IndividualLed[]::new));
    }

    public Color[] getColors() {
        return LedsManager.getInstance().getColors(offset, size);
    }

    public void setColor(Color[] colors) {
        setColor(IntStream.range(0, Math.min(colors.length, size)).mapToObj((i) -> new IndividualLed(i, colors[i]))
                .toArray(IndividualLed[]::new));
    }
}
