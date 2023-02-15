package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.chassis.Chassis;
import frc.robot.subsystems.chassis.ChassisConstants;
import frc.robot.utils.Utils;

public class KeepPosition extends CommandBase {
    private final Chassis chassis;
    private final Pose2d position;
    private final PIDController xPIDController, yPIDController, rotationPIDController;

    public KeepPosition(Chassis chassis, Pose2d position) {
        this.chassis = chassis;
        this.position = position;
        xPIDController = new PIDController(ChassisConstants.AUTO_TRANSLATION_KP, ChassisConstants.AUTO_TRANSLATION_KI, 0);
        yPIDController = new PIDController(ChassisConstants.AUTO_TRANSLATION_KP, ChassisConstants.AUTO_TRANSLATION_KI, 0);
        rotationPIDController = new PIDController(ChassisConstants.AUTO_ROTATION_KP, ChassisConstants.AUTO_ROTATION_KI, 0);
        rotationPIDController.enableContinuousInput(0, 2 * Math.PI);

        addRequirements(chassis);
    }

    @Override
    public void initialize() {
        xPIDController.setSetpoint(position.getX());
        yPIDController.setSetpoint(position.getY());
        rotationPIDController.setSetpoint(position.getRotation().getRadians());
    }

    @Override
    public void execute() {
        Pose2d currentPose = chassis.getPose();
        chassis.setVelocities(xPIDController.calculate(currentPose.getX()), yPIDController.calculate(currentPose.getY()), rotationPIDController.calculate(currentPose.getRotation().getRadians()));
        SmartDashboard.putNumber("KP X Error", currentPose.getX() - position.getX());
        SmartDashboard.putNumber("KP Y Error", currentPose.getY() - position.getY());
        SmartDashboard.putNumber("KP Degrees Error", Utils.getAngleDifference(position.getRotation().getDegrees(), currentPose.getRotation().getDegrees()));
    }

    @Override
    public void end(boolean interrupted) {
        chassis.stop();
    }
}
