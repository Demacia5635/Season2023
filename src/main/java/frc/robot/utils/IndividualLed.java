package frc.robot.utils;

import edu.wpi.first.wpilibj.util.Color;

public class IndividualLed {
    public final int index;
    public final Color color;

    public IndividualLed(int index, Color color) {
        this.index = index;
        this.color = color;
    }
}
