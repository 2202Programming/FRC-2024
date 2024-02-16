// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import frc.robot.Constants.Intake_Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.BlinkyLights.BlinkyLightUser;
import frc.robot.subsystems.Intake;


public class IntakeTest extends BlinkyLightUser {
    final static int DONE_COUNT = 100; // frames we expect to have note before finished

    /** Creates a new intakeForward. */
    public final Intake intake;
    double original_pos = 0;
    double count = 0;
    boolean vel_change = false;

    public IntakeTest() {
        this.intake = RobotContainer.getSubsystem(Intake.class);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        intake.setAngleVelocity(0.1);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        intake.setAngleVelocity(0.0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
