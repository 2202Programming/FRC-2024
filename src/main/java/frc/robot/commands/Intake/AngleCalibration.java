// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake;

public class AngleCalibration extends  Command {
    //Safe speed for moving to limit switch

    /** Creates a new intakeForward. */
    public final Intake intake;
    double angleVelocity; //[deg/s]
    public AngleCalibration(double angleVelocity) {
        this.intake = RobotContainer.getSubsystem(Intake.class);
        this.angleVelocity = angleVelocity;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        System.out.println("*******SPEED******" + angleVelocity);
        intake.setAngleVelocity(angleVelocity);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {        
        intake.setAngleVelocity(0.0);             
        if (interrupted) return;

        //made it here, then we completed calibration, so set our position
        intake.setAnglePosition( (angleVelocity < 0.0) ? Intake.UpPos : Intake.DownPos);
        intake.setAngleSetpoint(intake.getAnglePosition());
    }

    // Returns true when the command should end, we end when count hits DONE_COUNT
    @Override
    public boolean isFinished() {
        return  (angleVelocity < 0.0) ? intake.atReverseLimitSwitch() : intake.atForwardLimitSwitch();      
        
    }
}
