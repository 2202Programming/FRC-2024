// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.NoseRoller;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.Roller_Constants;
import frc.robot.subsystems.NoseRoller;

/**
 * Automate as much as possible
 * Two different positions - Default/transfer & Position to 'shoot'
 * Steps:
 * 1. Driver presses button
 * 2. Move nose roller to proper position
 * 2a. If interrupted, go back to original position
 * 3. Turn on motors that shoot
 * 4. Once finished shooting, turn motors off
 * 5. Return to correct position
 *
 */
public class NoseRollerToggle extends Command {
    /** Creates a new NoseRoller. */
    public final NoseRoller noseRoller;
    boolean turned_on;
    // double roller_speed;
    // double pos_cmd;

    public NoseRollerToggle(double pos_cmd) {
        // this.pos_cmd = pos_cmd;
        this.noseRoller = RobotContainer.getSubsystem(NoseRoller.class);
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        noseRoller.setNosePosition(Roller_Constants.RollerPosShoot);
        turned_on = false;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (noseRoller.isAtAngle() && !turned_on) {
            noseRoller.setRollerSpeed(Roller_Constants.RollerSpeedDefault);
            turned_on = true;
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            noseRoller.setRollerSpeed(0.0);
            noseRoller.setNosePosition(Roller_Constants.RollerPosDefault);
        }
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (!noseRoller.rollerHasNote()) {
            return true;
        }
        return false;
    }
}
