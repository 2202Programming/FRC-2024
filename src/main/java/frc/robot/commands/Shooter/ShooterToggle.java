// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.BlinkyLights;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Transfer;
import frc.robot.subsystems.BlinkyLights.BlinkyLightUser;

public class ShooterToggle extends Command {
  public final Shooter shooter;
  public final Intake intake;
  public final Transfer transfer;
  final int DELAY = 20; // figure out this number
  final int shooterTolerance = 100;
  Timer timer = new Timer();
  boolean RPM_dropped;
  int aprilTarget;
  private boolean startedShooting;

  /**
   * Wait until shooter is at desired RPM and then start the transfer motor.
   * Switch to ShooterSequence after test. Do this before adding pneumatics
   */
  @Deprecated
  public ShooterToggle() {
    this.shooter = RobotContainer.getSubsystem(Shooter.class);
    this.intake = RobotContainer.getSubsystem(Intake.class);
    this.transfer = RobotContainer.getSubsystem(Transfer.class);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startedShooting = false;
    RPM_dropped = false;
    if (transfer.hasNote()) {
      shooter.setRPM(0.5, 0.5);
    } else {
      end(true);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (shooter.isAtRPM(shooterTolerance) && intake.angleAtSetpoint() && !startedShooting) {// Check the RPM tolerance
      transfer.transferMotorOn();
      timer.restart();
    }
  }
      /*
     * Control the blinkylights based on our Note possession.
     *
     * Green when we have it, red otherwise
     */
    @Override
    public Color8Bit colorProvider() {
        // make sure not is safely in our possession before going back
        return (count >= DONE_COUNT) ? BlinkyLights.GREEN : BlinkyLights.RED;
    };

    @Override
    public boolean requestBlink() {
        return false; // we want solid lights
    }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.setRPM(0, 0);
    transfer.transferMotorOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (!shooter.isAtRPM(shooterTolerance)) {
      RPM_dropped = true;
    }
    // TODO:Just have this dealay for now, when we get sensors on add the sensor
    // check
    if (timer.hasElapsed(DELAY)) {
      return true;
    } else {
      return false;
    }
  }
}
