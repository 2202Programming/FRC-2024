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
    double AngleVelocity; //[deg/s]
    public AngleCalibration(double AngleVelocity) {
        this.intake = RobotContainer.getSubsystem(Intake.class);
        this.AngleVelocity = AngleVelocity;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        System.out.println("*******SPEED******" + AngleVelocity);
        intake.setAngleVelocity(AngleVelocity);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // intake.setAnglePosition(0.0);
        intake.setAngleSetpoint(intake.getAnglePosition());
      intake.setAngleVelocity(0.0);      
    //   intake.setAnglePosition(Intake_Constants.DrivingPosition);
    }

    // Returns true when the command should end, we end when count hits DONE_COUNT
    @Override
    public boolean isFinished() {
        return false;
        // return intake.atReverseLimitSwitch();      
    }
}
