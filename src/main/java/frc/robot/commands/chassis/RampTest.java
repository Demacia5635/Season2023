package frc.robot.commands.chassis;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.chassis.Chassis;
import frc.robot.utils.UtilsGeneral;

public class RampTest extends CommandBase {
    private final Chassis chassis;
    private final static double START_VEL = 1.8;
    private final static double MIN_ANGLE = 12;
    private double velocity;
    private boolean onRamp;
    private Timer timer;
    private Timer timer2;
    private double sign;
    private int phase;

    public RampTest(Chassis chassis) {
        this.chassis = chassis;
    }

    @Override
    public void initialize() {
        velocity = UtilsGeneral.isRedAlliance() ? START_VEL : -START_VEL;
        onRamp = false;
        phase = 0;
        timer = new Timer();
        timer2 = new Timer();
    }

    @Override
    public void execute() {
        if (!onRamp && Math.abs(chassis.getUpRotation()) > MIN_ANGLE) {
            onRamp = true;
            velocity /= 1.5;
            timer.start();
            sign = Math.signum(chassis.getUpAngularVel());
            phase = 1;
        } else if (onRamp && timer.get() >= 1 && phase == 1) {
            System.out.println("phase 1");
            phase = 2;
            velocity /= 3.5;
        } else if (onRamp && timer.get() >= 2 && phase == 2) {
            System.out.println("phase 2");
            phase = 3;
            velocity /= 4.5;
            timer2.start();
        }else if ( phase == 3 && -chassis.getUpAngularVel() * sign >= 10) {
            System.out.println("phase 3");
            phase = 4;
            velocity = 0;
        }
        if (timer != null)
            SmartDashboard.putNumber("Ramp/Timer", timer.get());
        SmartDashboard.putNumber("Ramp/Angle", chassis.getUpRotation());
        SmartDashboard.putNumber("Ramp/AngularVel", chassis.getUpAngularVel());
        SmartDashboard.putBoolean("Ramp/ONRAMP", onRamp);
        chassis.setAngleAndVelocity(velocity, 0, UtilsGeneral.isRedAlliance() ? 0 : Math.toRadians(180));
    }

    
    @Override
    public boolean isFinished() {
        return (onRamp && phase == 4) || (onRamp && timer2.get() > 0.5);
    }

    @Override
    public void end(boolean interrupted) {
        chassis.stop();
    }
}
