package frc.robot.commands.chassis;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.chassis.Chassis;
import frc.robot.subsystems.chassis.ChassisConstants;
import frc.robot.utils.UtilsGeneral;

/**
 * A command that keeps the robot at a certain position on the field.
 */
public class KeepPosition extends CommandBase {
    private final Chassis chassis;
    private final Pose2d position;
    private final PIDController xPIDController, yPIDController, rotationPIDController;

    /**
     * Creates a new KeepPosition command.
     * 
     * @param chassis The chassis subsystem
     * @param position The position to keep
     */
    public KeepPosition(Chassis chassis, Pose2d position) {
        this.chassis = chassis;
        this.position = position;
        xPIDController = new PIDController(ChassisConstants.AUTO_TRANSLATION_KP, ChassisConstants.AUTO_TRANSLATION_KI, 0);
        yPIDController = new PIDController(ChassisConstants.AUTO_TRANSLATION_KP, ChassisConstants.AUTO_TRANSLATION_KI, 0);
        rotationPIDController = new PIDController(ChassisConstants.AUTO_ROTATION_KP, ChassisConstants.AUTO_ROTATION_KI, 0);
        rotationPIDController.enableContinuousInput(0, 2 * Math.PI);

        xPIDController.setTolerance(ChassisConstants.AUTO_TRANSLATION_TOLERANCE);
        yPIDController.setTolerance(ChassisConstants.AUTO_TRANSLATION_TOLERANCE);
        rotationPIDController.setTolerance(ChassisConstants.AUTO_ANGLE_TOLERANCE);

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
        double vx, vy, vo;
        if (xPIDController.atSetpoint())
            vx = 0;
        else
            vx = xPIDController.calculate(currentPose.getX());
        if (yPIDController.atSetpoint())
            vy = 0;
        else
            vy = yPIDController.calculate(currentPose.getY());
        if (rotationPIDController.atSetpoint())
            vo = 0;
        else
            vo = rotationPIDController.calculate(currentPose.getRotation().getRadians());
        chassis.setVelocities(vx, vy, vo);
        SmartDashboard.putNumber("KP X Error", currentPose.getX() - position.getX());
        SmartDashboard.putNumber("KP Y Error", currentPose.getY() - position.getY());
        SmartDashboard.putNumber("KP Degrees Error", UtilsGeneral.getAngleDifference(position.getRotation().getDegrees(), currentPose.getRotation().getDegrees()));
    }

    @Override
    public boolean isFinished() {
        return xPIDController.atSetpoint() && yPIDController.atSetpoint() && rotationPIDController.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        chassis.softwareStop();
    }
}
