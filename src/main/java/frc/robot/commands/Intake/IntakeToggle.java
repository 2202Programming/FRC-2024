// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Intake_Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.BlinkyLights;
import frc.robot.subsystems.BlinkyLights.BlinkyLightUser;
import frc.robot.subsystems.Intake;

/**
 * 
 * Automate as much of intake as possible and pass off to transfer system
 * or maybe hold Note to deliver through nose-roller.
 * 
 * Basic operation:
 * 1: Driver holds button
 * 2: Set the intake speed to the desired speed
 * 3: set the intake angle to the desired angle (floor pick up angle)
 * Blink lights RED or TBD when looking for a NOTE
 * 
 * 4a. If driver releases button,
 * set back to wheel speed of 0 and original intake angle
 * 
 * 4b. If a note is detected, wait x amount of time to position Note
 * and set the wheel speed to 0. This holds it for next step.
 * This makes sure that the note is secure by waiting x amt of time after lightgate detects
 * Blink Lights GREEN when we have the note
 * Bring the angle to the transfer position
 * 
 * 4c. If note is detected but then is "undetected"
 * after time of it being gone just give up.
 * 
 * Blink Lights RED if we lose the note.
 * 
 * 
 */
public class IntakeOn extends Command implements BlinkyLightUser {
  final static int DONE_COUNT = 5; // frames we expect to have note before finished

  /** Creates a new intakeForward. */
  public final Intake intake;
  double intake_speed;
  boolean use_dashboard = false;
  double angle_cmd;
  double original_pos = 0;
  double count = 0;

  public IntakeOn(double intake_speed, double angle_cmd) {
        this.intake = RobotContainer.getSubsystem(Intake.class);
    this.intake_speed = intake_speed;
    this.angle_cmd = angle_cmd;
  }

  public IntakeOn() {
    // use default
    this(Intake_Constants.IntakeMotorDefault, Intake_Constants.AngleMotorDefault);
  }

  // Should mostly be using same speed, which is why constructor for only speed is
  // present, but not pos
  public IntakeOn(double angle_pos) {
    this(Intake_Constants.IntakeMotorDefault, angle_pos);
    // default speed
  }

  public IntakeOn(boolean use_dashboard) {
    this();
    if (use_dashboard) {
      this.intake_speed = SmartDashboard.getNumber("Intake Speed", Intake_Constants.IntakeMotorDefault);
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    original_pos = intake.getAnglePosition();
    intake.setAngleSetpoint(angle_cmd);
    intake.setMotorSpeed(intake_speed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (use_dashboard) {
      intake_speed = SmartDashboard.getNumber("Intake Speed", 0);
      intake.setMotorSpeed(intake_speed);
    }
    // blinky lights controlled in

    // maybe unneeded code
    if (count > DONE_COUNT) {
      intake.setMotorSpeed(0.0);
    }
  }

  /*
   * Control the blinkylights based on our Note possession.
   * 
   * Green when we have it, red otherwise
   */
  @Override
  public Color8Bit colorProvider() {
    return (intake.hasNote()) ? BlinkyLights.GREEN : BlinkyLights.RED;
  };

  @Override
  public boolean requestBlink() {
    return false; // we want solid lights
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // always stop the motor
    intake.setMotorSpeed(0.0);

    // driver gave up or we have the note
    if (interrupted || count < DONE_COUNT) {
      //note: why is this not just transfer pos anyway?? - is this if statement needed or just no matter what we set to transfer pos
      intake.setAngleSetpoint(original_pos);
    } else {
      intake.setAngleSetpoint(Intake_Constants.TransferPosition);
    }
  }

  // Returns true when the command should end, we end when count hits DONE_COUNT
  @Override
  public boolean isFinished() {
    count = (intake.hasNote()) ? count++ : 0; // count frames with note
    return (count >= DONE_COUNT);// done when we hit the count
  }
}
