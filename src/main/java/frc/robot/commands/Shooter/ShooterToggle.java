// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter; 

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Transfer;
import frc.robot.subsystems.Swerve.SwerveDrivetrain;

public class ShooterToggle extends Command {
  /** Creates a new ShooterToggle. */
  public final Shooter shooter;
  public final Intake intake;
  public final Transfer transfer;
  final boolean pneumatics = false; // YES FOR SUSSEX NO AFTER???
  final int DELAY = 20; // figure out this number
  int count = 0;
  boolean RPM_dropped;
  int aprilTarget;
  private final SwerveDrivetrain drivetrain;

  //final Intake intake; //TODO: When merge, check for hasNote - Probably move to subsystem
  public ShooterToggle(String target) {
    this.shooter = RobotContainer.getSubsystem(Shooter.class);
    this.intake = RobotContainer.getSubsystem(Intake.class);
    this.transfer = RobotContainer.getSubsystem(Transfer.class);
    this.drivetrain = RobotContainer.getSubsystem(SwerveDrivetrain.class);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RPM_dropped = false;
   if(intake.hasNote()){
    shooter.setRPM(0.5, 0.5);
  } else {
    isFinished();
  }
  } 

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //to simplify the code im just using the fixed pose and assuming on red alliance but this will be changed
    Pose2d targetPose = new Pose2d(new Translation2d(14.76, 5.48), new Rotation2d(171.50));
    if( //If statement to avoid the zero autopath bugs
      Math.sqrt(Math.pow(drivetrain.getPose().getX() - targetPose.getX(), 2) + Math.pow(drivetrain.getPose().getY() - targetPose.getY(),2))
      > 0.5//Magic number for distance to target
      && Math.abs(drivetrain.getPose().getRotation().getDegrees() - targetPose.getRotation().getDegrees()) > 5
      ){
        AutoBuilder.pathfindToPose(new Pose2d(new Translation2d(1.73, 5.38), new Rotation2d(0.0)),
          new PathConstraints(3.0, 3.0, Units.degreesToRadians(540), Units.degreesToRadians(720)));
    }
    if(shooter.isAtRPM() && intake.angleAtSetpoint() && Math.sqrt(Math.pow(drivetrain.getPose().getX() - targetPose.getX(), 2) + Math.pow(drivetrain.getPose().getY() - targetPose.getY(),2))
      < 0.5//Magic number for distance to target
      && Math.abs(drivetrain.getPose().getRotation().getDegrees() - targetPose.getRotation().getDegrees()) < 5){
      transfer.transferMotorOn();
    }

  }
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.setRPM(0, 0);
    // shooter.setTransferOff(); once merged (might be transfer.transferMotorOff())
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(Math.abs(shooter.getRPM() - shooter.getDesiredRPM()) > 100){
      RPM_dropped = true;
    } if(RPM_dropped){
      count++;
    } if(count > DELAY){
      return true;
    } else {
      return false;
    }
  }
}
