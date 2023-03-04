package frc.robot.commands.parallelogram;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.parallelogram.ParallelConstants;
import frc.robot.subsystems.parallelogram.Parallelogram;

public class CalibrateParallelogram extends CommandBase {
    private Parallelogram parallelogram;
    private double testAngle;
    private double diff;
    
    /**
     * Command's constructor.
     * @param parallelogram
     */
    public CalibrateParallelogram(Parallelogram parallelogram) {
        this.parallelogram = parallelogram;
        addRequirements(parallelogram);
        diff = 0;
    }


    @Override
    public void initialize() {
        testAngle = parallelogram.getAngle();
        parallelogram.setBrake();
        parallelogram.setPower(ParallelConstants.CALIBRATION_POWER);
    }

    @Override
    public void execute(){
        diff = 129- parallelogram.getAngle();
        if(parallelogram.getAngle() > 90){
            parallelogram.setPower((diff*0.01) + 0.05 > ParallelConstants.CALIBRATION_POWER? ParallelConstants.CALIBRATION_POWER : (diff*0.01) + 0.05);
        }
    }


    @Override
    public boolean isFinished() {
        System.out.println("Activation Value: " + testAngle +  " Current Angle: " + parallelogram.getAngle() + " Desired Angle: " );
        return parallelogram.getDigitalInput();
    }

    @Override
    public void end(boolean interrupted) {
        parallelogram.setPower(0);
    }
}
