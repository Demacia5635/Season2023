package frc.robot.commands.parallelogram;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.chassis.Chassis;
import frc.robot.subsystems.parallelogram.ParallelConstants;
import frc.robot.subsystems.parallelogram.Parallelogram;

public class ResetCalibrate extends CommandBase {
    private Parallelogram parallelogram;
    private Chassis chassis;

    /**
     * Command's constructor.
     * @param parallelogram
     */
    public ResetCalibrate(Parallelogram parallelogram, Chassis chassis) {
        this.parallelogram = parallelogram;
        this.chassis = chassis;
        addRequirements(parallelogram);
    }


    @Override
    public void initialize() {
        parallelogram.setBrake();
    }

    @Override
    public void execute() {
        double rawVelocity = -chassis.getVelocity().getX();
        if (rawVelocity>2.5) {
            SmartDashboard.putBoolean("yes", true);
            parallelogram.setPower(ParallelConstants.BACKWARDS_CALIBRATION_POWER + 0.1);
        }
        else {
            parallelogram.setPower(ParallelConstants.BACKWARDS_CALIBRATION_POWER);
        }
    }

    @Override
    public boolean isFinished() {
        return !parallelogram.getDigitalInput();
    }

    @Override
    public void end(boolean interrupted) {
        parallelogram.resetPosition();
        parallelogram.setPower(0);
    }
}