package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.Limelight;
import frc.lib.util.Limelight.ledStates;
import frc.robot.Constants;

public class Vision extends SubsystemBase {
    public Limelight limelight;

    public Vision() {
        limelight = new Limelight(
            Constants.Vision.limelightHeight, 
            Constants.Vision.limelightAngle, 
            Constants.Vision.goalHeight
        );
    }    
    
    @Override
    public void periodic(){
        limelight.ledState(ledStates.off);
        SmartDashboard.putNumber("LLDistance", limelight.getDistance().getNorm());
    }

}