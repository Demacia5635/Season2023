package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.led_patches.SubStrip;

public class CarryTarget extends CommandBase {
    private final SubStrip strip;

    public CarryTarget(SubStrip strip) {
        this.strip = strip;
        addRequirements(strip);
    }
}
