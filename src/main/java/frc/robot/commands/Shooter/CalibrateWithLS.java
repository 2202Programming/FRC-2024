// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.ShooterServo;

public class CalibrateWithLS extends  Command {
    //Safe speed for moving to limit switch

    /** Creates a new intakeForward. */
    boolean done;
    public final ShooterServo shooter;
    double angleVelocity; //[deg/s]
    int count;
    public enum Phase {
        Start, BeyondLS
      }
    Phase phase;
    public CalibrateWithLS(double angleVelocity) {
        this.shooter = RobotContainer.getSubsystem(ShooterServo.class);
        this.angleVelocity = angleVelocity;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        phase = Phase.Start;
        shooter.setExtensionVelocity(angleVelocity);
        done = false;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        switch(phase){
            case Start:
            if(shooter.atLowLimit()){
                shooter.setExtensionVelocity(1.0);
            }
                if(!shooter.atLowLimit()){
                    phase = Phase.BeyondLS;
                }
                break;
             case BeyondLS:
                shooter.setExtensionVelocity(-1.0); 
            if(shooter.atLowLimit() &&!done){
            shooter.setExtensionVelocity(0.0);
            shooter.setAngleSetpoint(shooter.getAngle() - 3.818581);
            System.out.println(shooter.getAngle() - 3.818581);
            done = true;
        }  
        break;
        }
    }
    
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) { 
        shooter.setExtensionVelocity(0.0);
        shooter.setExtensionPosition(0.0);

    }

    // Returns true when the command should end, we end when count hits DONE_COUNT
    @Override
    public boolean isFinished() {
        if(done){
        return (shooter.atSetpoint());
        }
        return false;
        //for alpha
        // return  (angleVelocity < 0.0) ? intake.atReverseLimitSwitch() : intake.atForwardLimitSwitch();      
        
    }
}
