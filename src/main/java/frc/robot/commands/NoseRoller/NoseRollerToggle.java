// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.commands.NoseRoller;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
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
double roller_speed;
double original_pos;
double pos_cmd;


public NoseRollerToggle(double pos_cmd) {
this.pos_cmd = pos_cmd;
this.noseRoller = RobotContainer.getSubsystem(NoseRoller.class);
// Use addRequirements() here to declare subsystem dependencies.
}


// Called when the command is initially scheduled.
@Override
public void initialize() {
original_pos = noseRoller.getNosePosition();
}


// Called every time the scheduler runs while the command is scheduled.
@Override
public void execute() {
if(noseRoller.getNosePosition() == pos_cmd){
noseRoller.setMotorsToStart(roller_speed);
}
}


// Called once the command ends or is interrupted.
@Override
public void end(boolean interrupted) {}


// Returns true when the command should end.
@Override
public boolean isFinished() {
return false;
}
}


