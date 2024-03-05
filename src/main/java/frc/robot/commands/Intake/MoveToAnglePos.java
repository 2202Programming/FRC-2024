// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.RobotContainer;
import frc.robot.subsystems.BlinkyLights;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.BlinkyLights.BlinkyLightUser;

public class MoveToAnglePos extends BlinkyLightUser {
  /** Creates a new AnglePos. */
  public final Intake intake;
  double posCmd;
  double velLimit;
  
  public MoveToAnglePos(double posCmd, double velLimit) {
    this.intake = RobotContainer.getSubsystem(Intake.class);
    this.posCmd = posCmd;
    this.velLimit = velLimit;
    addRequirements(intake);
  }

      /*
     * Control the blinkylights based on being at calibration point or not.
     */
    @Override
    public Color8Bit colorProvider() {
        // green when moving down, red moving back
        return (intake.getAngleSpeed() >0.0) ? BlinkyLights.GREEN : BlinkyLights.RED;
    };

    @Override
    public boolean requestBlink() {
        return false; // we want solid lights
    }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.setMaxVelocity(velLimit);
    intake.setAngleSetpoint(posCmd);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setAngleVelocity(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return intake.angleAtSetpoint();
  }
}
