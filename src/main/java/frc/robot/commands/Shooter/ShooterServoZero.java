// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ShooterServo;

public class ShooterServoZero extends Command {
  /** Creates a new ShooterServoZero. */

    private final ShooterServo shooterServo;

    //TODO: reasonable tolerance
    private double velTol = 0.1;
    
    private boolean finished = false;

  //Lower the shooter servo until it stops, then set relative encoder to zero.

  public ShooterServoZero() {
    shooterServo = RobotContainer.getSubsystem(ShooterServo.class);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterServo);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooterServo.setShooterAngleVelocity(-0.2);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //TODO: may want to also watch current
    if (Math.abs(shooterServo.getShooterAngleVelocity()) <  velTol){
      finished = true;
    }
    SmartDashboard.putNumber("Shooter Servo Velocity", shooterServo.getShooterAngleVelocity());
    SmartDashboard.putNumber("Shooter Servo Position", shooterServo.getShooterAnglePosition());
    SmartDashboard.putNumber("Shooter Servo Current", shooterServo.getCurrent());
    System.out.println("Servo zeroing: velocity="+shooterServo.getShooterAngleVelocity()
      +" Postion=" + shooterServo.getShooterAnglePosition()
      +" Current=" + shooterServo.getCurrent());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterServo.setShooterAngleVelocity(0.0);
    shooterServo.zeroEncoder();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
