// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.Tag_Pose;
import frc.robot.subsystems.Swerve.SwerveDrivetrain;

public class RotateTo extends Command {
  private final SwerveDrivetrain drivetrain;
  private PIDController pid;
  private final double kp = 5.0;
  private final double ki = 0.0;
  private final double kd = 0.0;
  private final double pos_tol = 3.0;
  private SwerveDriveKinematics kinematics;
  private Pose2d currentPose;
  private double targetRot;
  private SwerveModuleState[] outputModuleState;
  private Translation2d targetPose;
  private Timer timer;

  /** Creates a new RotateTo. */
  public RotateTo() {
    drivetrain = RobotContainer.getSubsystem(SwerveDrivetrain.class);
    addRequirements(drivetrain);
    pid.setTolerance(pos_tol);
    pid = new PIDController(kp, ki, kd);
    kinematics = drivetrain.getKinematics();
    targetPose = (DriverStation.getAlliance().get() == Alliance.Blue) ? Tag_Pose.ID7 : Tag_Pose.ID4;
    timer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.restart();
    currentPose = drivetrain.getPose();
    if (currentPose.getX() - targetPose.getX() < 0) {//Adding 180 or not
      targetRot = 180 +
          (Math.atan2(currentPose.getTranslation().getX() - targetPose.getX(),
              currentPose.getTranslation().getY() - targetPose.getY())) // [-pi, pi]
              * 180 / Math.PI;
    } else {
      targetRot = (Math.atan2(currentPose.getTranslation().getX() - targetPose.getX(),
          currentPose.getTranslation().getY() - targetPose.getY())) // [-pi, pi]
          * 180 / Math.PI;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    calculate();
    drivetrain.drive(outputModuleState);
  }

  private void calculate() {
    currentPose = drivetrain.getPose();
    outputModuleState = kinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(
        0,
        0,
        pid.calculate(currentPose.getRotation().getDegrees(), targetRot),
        currentPose.getRotation()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pid.atSetpoint() || timer.hasElapsed(4);
  }
}
