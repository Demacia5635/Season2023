// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.LedConstants;
import frc.robot.commands.Flicker;
import frc.robot.commands.RollyPolly;
import frc.robot.commands.chassis.Drive;
import frc.robot.commands.chassis.GoToNodesHalfManual;
import frc.robot.commands.chassis.GotoCommunity;
import frc.robot.commands.chassis.GotoLoadingZone;
import frc.robot.commands.chassis.GotoNodes;
import frc.robot.commands.chassis.LeaveCommunity;
import frc.robot.subsystems.chassis.Chassis;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.gripper.GripperConstants;
import frc.robot.subsystems.led_patches.SubStrip;
import frc.robot.subsystems.parallelogram.Parallelogram;
import frc.robot.utils.IntPair;
import frc.robot.utils.UtilsGeneral;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    private static RobotContainer instance;
    private final CommandXboxController main = new CommandXboxController(0);
    public final CommandXboxController secondary = new CommandXboxController(1);
    private final Chassis chassis;
    private final Parallelogram parallelogram;
    private final Gripper gripper;
    private GenerateAutonomous generateAutonomous;
    private GotoNodes gotoNodes;
    private LeaveCommunity leaveCommunity;

    private final SubStrip rollStrip;
    private final SubStrip pitchStrip;
    private final SubStrip allStrip;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    private RobotContainer() {
        chassis = new Chassis();
        rollStrip = new SubStrip(new IntPair(65, 74));
        Command rollCommand = new RepeatCommand(
                new WaitUntilCommand(() -> Math.abs(chassis.getRoll()) > LedConstants.EPSILON).andThen(
                        new RollyPolly(rollStrip, chassis::getRoll, Color.kBlue, Color.kYellow, 0),
                        new Flicker(rollStrip)))
                .ignoringDisable(true);
        rollStrip.setDefaultCommand(rollCommand);
        rollCommand.schedule();
        pitchStrip = new SubStrip(new IntPair(139, 36), new IntPair(28, 36));
        Command pitchCommand = new RepeatCommand(
                new WaitUntilCommand(() -> Math.abs(chassis.getPitch()) > LedConstants.EPSILON)
                        .andThen(new RollyPolly(pitchStrip, chassis::getPitch, Color.kRed, Color.kGreen, 0),
                                new Flicker(pitchStrip)))
                .ignoringDisable(true);
        pitchStrip.setDefaultCommand(pitchCommand);
        pitchCommand.schedule();

        allStrip = new SubStrip(new IntPair(0, 175));
        allStrip.turnOff();

        parallelogram = new Parallelogram();
        chassis.setDefaultCommand(new Drive(chassis, main.getHID()));
        SmartDashboard.putData((Sendable) chassis.getDefaultCommand());
        SmartDashboard.putData(chassis);
        gripper = new Gripper(GripperConstants.MOTOR_ID);
        gotoNodes = new GotoNodes(chassis, secondary, parallelogram);
        leaveCommunity = new LeaveCommunity(chassis);
        SmartDashboard.putData(gripper);
        configureButtonBindings();

        SmartDashboard.putData(CommandScheduler.getInstance());
        SmartDashboard.putBoolean("is left led", false);

        generateAutonomous = new GenerateAutonomous(gotoNodes, leaveCommunity, gripper, parallelogram, chassis);
    }

    /**
     * Returns the instance of the RobotContainer class.
     * 
     * @return the instance of the RobotContainer class.
     */
    public static RobotContainer getInstance() {
        if (instance == null) {
            instance = new RobotContainer();
        }
        return instance;
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link Joystick}
     * or {@link XboxController}), and then passing it to a {@link JoystickButton}.
     */
    private void configureButtonBindings() {

        Command load = gripper.getOpenCommand().alongWith(new GotoLoadingZone(chassis, secondary,
                parallelogram.getGoToAngleCommand(Constants.LOADING_ANGLE)))
                .andThen(gripper.getCloseCommand());

        Command unload = new GotoCommunity(chassis)
                .andThen(new GoToNodesHalfManual(chassis, secondary, parallelogram));

        load = load.until(() -> UtilsGeneral.hasInput(main.getHID()))
                .andThen((new InstantCommand(() -> parallelogram.getGoBackCommand().schedule())));
        unload = unload.until(() -> UtilsGeneral.hasInput(main.getHID()))
               /*.andThen(new InstantCommand(() -> parallelogram.getGoBackCommand().schedule()))*/;

        main.leftBumper().onTrue(new InstantCommand(() -> gripper.getCloseCommand().schedule()));
        main.rightBumper().onTrue(new InstantCommand(() -> gripper.getOpenCommand().schedule()));

        load.setName("Load");

        unload.setName("Unload");

        main.a().onTrue(load);
        main.x().onTrue(unload);
        main.y().onTrue(new InstantCommand(() -> parallelogram.getGoBackCommand().schedule()));
        main.povRight().onTrue(parallelogram.getGoToAngleCommand(Constants.DEPLOY_ANGLE));
        main.povUp().onTrue(parallelogram.getGoToAngleCommand(Constants.LOADING_ANGLE));
        main.povDown().onTrue(new StartEndCommand(chassis::setRampPosition, chassis::stop, chassis)
                .until(() -> UtilsGeneral.hasInput(main.getHID())));
        main.povLeft().onTrue(parallelogram.getGoToAngleCommand(Constants.DEPLOY_HIGH_CUBES1));

        secondary.back().and(secondary.start())
                .whileTrue(new RunCommand(() -> CommandScheduler.getInstance().cancelAll()));

        secondary.leftBumper().and(secondary.rightBumper())
                .onTrue(new InstantCommand(() -> {
                    if (allStrip.getColors()[0].equals(Color.kRed))
                        allStrip.turnOff();
                    else
                        allStrip.setColor(Color.kRed);
                }).ignoringDisable(true));
        secondary.leftBumper().and(secondary.rightBumper().negate())
                .onTrue(new InstantCommand(() -> {
                    Color color = new Color(168, 0, 230);
                    if (allStrip.getColors()[0].equals(color))
                        allStrip.turnOff();
                    else
                        allStrip.setColor(color);
                }).ignoringDisable(true));
        secondary.rightBumper().and(secondary.leftBumper().negate())
                .onTrue(new InstantCommand(() -> {
                    Color color = new Color(255, 140, 0);
                    if (allStrip.getColors()[0].equals(color))
                        allStrip.turnOff();
                    else
                        allStrip.setColor(color);
                }).ignoringDisable(true));

    }

    /*
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    //  TODO: RETURN NORAML AUTO COMMAN
    public Command getAutonomousCommand() {
       return generateAutonomous.getAutonomous();
    }

    public void onTeleopInit() {
        chassis.getDefaultCommand().schedule();
        new StartEndCommand(()->parallelogram.setPower(-0.3), ()->{parallelogram.setPower(0);}, parallelogram)
            .withTimeout(0.4).andThen(
        new InstantCommand(()->parallelogram.getCalibrationCommand(chassis).schedule())).schedule();
    }

    public void onEnableInit(){
        chassis.setBreak();
    }

    public void setChassisCoast(){
        chassis.setCoast();
    }
}