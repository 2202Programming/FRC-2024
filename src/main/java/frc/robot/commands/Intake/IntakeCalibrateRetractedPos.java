// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants.Intake_Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.BlinkyLights;
import frc.robot.subsystems.BlinkyLights.BlinkyLightUser;
import frc.robot.subsystems.Intake;

public class IntakeCalibrateRetractedPos extends  BlinkyLightUser {
    //Safe speed for moving to limit switch
    final static double AngleVelocity = 5.0; //[deg/s] 

    /** Creates a new intakeForward. */
    public final Intake intake;

    public IntakeCalibrateRetractedPos() {
        this.intake = RobotContainer.getSubsystem(Intake.class);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        intake.setAngleVelocity(AngleVelocity);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    /*
     * Control the blinkylights based on being at calibration point or not.
     */
    @Override
    public Color8Bit colorProvider() {
        // make sure not is safely in our possession before going back
        return (intake.atLimitSwitch()) ? BlinkyLights.GREEN : BlinkyLights.RED;
    };

    @Override
    public boolean requestBlink() {
        return false; // we want solid lights
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
      intake.setAngleVelocity(0.0);      
      intake.setAnglePosition(Intake_Constants.DrivingPosition);
    }

    // Returns true when the command should end, we end when count hits DONE_COUNT
    @Override
    public boolean isFinished() {
        return intake.atLimitSwitch();      
    }
}
