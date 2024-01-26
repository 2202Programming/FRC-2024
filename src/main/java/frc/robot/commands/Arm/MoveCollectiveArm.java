// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSS;

public class MoveCollectiveArm extends Command {
  /** Creates a new MoveCollectiveArm. */
  ArmSS arm = RobotContainer.RC().armSS;
  Positions start;
  Positions target;
  double old_arm_max_vel;
  boolean heading_out;
  public static class Positions{
    public double armPos;
    public double armMaxVel;

    public Positions(double armPos){
      this(armPos);
    }
    // shouldn't need?
    // public Postions(double armPos, double armVel){
    //   this.armPos = armPos;
    //   this.armMaxVel = armVel;
    // }
    // public Positions(Positions src){
    //   this(src.armPos, src.armMaxVel);
    // }
    // public Positions(CollectivePositions src){
    //   this(src.pos_info);
    // }
  }
  public MoveCollectiveArm(CollectivePositions Destination) 
  {
    this(Destination);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    old_arm_max_vel = arm.getMaxVel();
    if(target.armMaxVel > 0.0){
      arm.setMaxVel(target.armMaxVel);
    }
    start = getStart();
    heading_out = (target.armPos - start.armPos) > 0.0;
    arm.setSetpoint(target.armPos);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.setMaxVel(old_arm_max_vel);
    }
  Positions getStart(){
    return new Positions(
      arm.getPosition());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return arm.atSetpoint();
  }
}
