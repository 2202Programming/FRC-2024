// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ShooterServo;

public class CalibrateWithLS extends Command {
    // Safe speed for moving to limit switch

    /** Creates a new intakeForward. */
    public final ShooterServo shooter;
    double angleVelocity; // [deg/s]
    int count;
    final int DELAY_COUNT = 15;

    public enum Phase {
        Start, BeyondLS, Finished
    }

    Phase phase;

    public CalibrateWithLS(double angleVelocity) {
        this.shooter = RobotContainer.getSubsystem(ShooterServo.class);
        this.angleVelocity = angleVelocity;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        count = 0;
        phase = Phase.Start;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        switch (phase) {
            case Start:
                if (shooter.atLowLimit()) {
                    shooter.setExtensionVelocity(6.0);
                }
                if (!shooter.atLowLimit()) {
                    count++;
                }
                if(count >=DELAY_COUNT){
                                        phase = Phase.BeyondLS;
                }
                break;
            case BeyondLS:
                shooter.setExtensionVelocity(-1.0);
                if (shooter.atLowLimit()) {
                    shooter.setExtensionVelocity(0.0);
                    phase = Phase.Finished;
                }
                break;
            case Finished:
            break;
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        shooter.setExtensionPosition(1.68);

    }

    // Returns true when the command should end, we end when count hits DONE_COUNT
    @Override
    public boolean isFinished() {
        if (phase == Phase.Finished) {
            return (shooter.atSetpoint());
        }
        return false;
        // for alpha
        // return (angleVelocity < 0.0) ? intake.atReverseLimitSwitch() :
        // intake.atForwardLimitSwitch();

    }
}