// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Tag_Pose;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.ShooterServo;
import frc.robot.subsystems.Swerve.SwerveDrivetrain;
import frc.robot.util.DistanceInterpretor;

public class ShooterAutoAngle extends Command {

  private double distanceToTarget;
  private double targetAngle;
  private DistanceInterpretor distanceInterpretor;
  private Translation2d targetTranslation2d;
  private final ShooterServo shooterServo;

  public ShooterAutoAngle(Translation2d target) {
    targetTranslation2d = target;
    shooterServo = RobotContainer.getSubsystem(ShooterServo.class);
    addRequirements(shooterServo); // we are the captain now
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    distanceInterpretor = new DistanceInterpretor();
    targetTranslation2d = (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) ? 
      Tag_Pose.ID7 : // Blue Alliance
      Tag_Pose.ID4; // Red Alliance
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    calculateTargetAngle();
    if (RobotContainer.getSubsystem(Intake.class).hasNote()) {
      shooterServo.setAngleSetpoint(targetAngle); // has note, set angle based on distance to target
    } else {
      shooterServo.setAngleSetpoint(ShooterServo.MIN_DEGREES); // no note, keep shooter low to promote note intake
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  private void calculateTargetAngle() {
    distanceToTarget = RobotContainer.getSubsystem(SwerveDrivetrain.class)
        .getDistanceToTranslation(targetTranslation2d);
    targetAngle = distanceInterpretor.getAngleFromDistance(distanceToTarget);

    SmartDashboard.putNumber("Distance to Target", distanceToTarget);
    SmartDashboard.putNumber("Goal Angle for target", targetAngle);

  }

}
