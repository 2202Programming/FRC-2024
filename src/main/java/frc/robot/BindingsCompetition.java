package frc.robot;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Climber.Climb;
import frc.robot.commands.Climber.ClimberVelocity;
import frc.robot.commands.Intake.AngleCalibration;
import frc.robot.commands.Intake.EjectNote;
import frc.robot.commands.Intake.InIntake;
import frc.robot.commands.Intake.IntakeSequence;
import frc.robot.commands.Intake.MoveToAnglePos;
import frc.robot.commands.Intake.TestIntake;
import frc.robot.commands.Shooter.ShooterAngleSetPos;
import frc.robot.commands.Shooter.ShooterAngleVelMove;
import frc.robot.commands.Shooter.ShooterServoSequence;
import frc.robot.commands.Swerve.AllianceAwareGyroReset;
import frc.robot.commands.Swerve.RobotCentricDrive;
import frc.robot.commands.auto.AutoShooting;
import frc.robot.commands.auto.AutoShooting.ShootingTarget;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.ShooterServo;
import frc.robot.subsystems.Swerve.SwerveDrivetrain;
import frc.robot.subsystems.hid.HID_Xbox_Subsystem;

/*
 * Please don't edit this without leads/mentor/driveteam review
 */
public final class BindingsCompetition {

    public static void ConfigueCompetition(HID_Xbox_Subsystem dc) {
        DriverBinding(dc);
        OperatorBindings(dc);
    }


    static void DriverBinding(HID_Xbox_Subsystem dc) {
        var driver = dc.Operator();
        var drivetrain = RobotContainer.getSubsystem(SwerveDrivetrain.class);

        // Driver buttons
        driver.leftBumper().whileTrue(new RobotCentricDrive(drivetrain, dc));
        driver.y().onTrue(new AllianceAwareGyroReset(true));
    }


    static void OperatorBindings(HID_Xbox_Subsystem dc) {
        var sideboard = dc.SwitchBoard();
        var operator = dc.Operator();

        var climber = RobotContainer.getSubsystem(Climber.class);
        var shooter = RobotContainer.getSubsystem(ShooterServo.class);

        Trigger ManualShoot = sideboard.sw16();
        Trigger ClimberCalibrate = sideboard.sw11();
        Trigger ShooterCalibrate = sideboard.sw12();
        Trigger IntakeCalibrate = sideboard.sw13();

        // Switchboard buttons too
        sideboard.sw21().onTrue(new Climb(Climber.ExtendPosition));
        sideboard.sw22().onTrue(new Climb(Climber.ClimbPosition));
        sideboard.sw23().onTrue(new MoveToAnglePos(Intake.DownPos, Intake.TravelUp));

        /***************************************************************************************/
        // REAL COMPETITION BINDINGS.
        operator.a().whileTrue(new IntakeSequence(false)
                .andThen(new ShooterAngleSetPos(36.0)));
        operator.b().whileTrue(new EjectNote()); // eject note from intake
        operator.x().whileTrue(new InIntake(false)); // works ---> seq for stay in intake for amp shoot
        IntakeCalibrate.and(operator.povUp()).onTrue(new AngleCalibration(-25.0));// intake calibrate
        IntakeCalibrate.and(operator.povDown()).whileTrue(new TestIntake(0.0));

        ManualShoot.and(operator.rightBumper()).onTrue(new ShooterServoSequence(46.5, 2800.0));                                                                                                
        ManualShoot.and(operator.rightTrigger()).onTrue(new ShooterServoSequence(36.0, 3000.0)); // was 35
        // AutoShootm 
        ManualShoot.negate().and(operator.rightBumper())
            .onTrue(new AutoShooting(ShootingTarget.Speaker, 45.0, 3000.0));
        ManualShoot.negate().and(operator.rightTrigger())
            .onTrue(new AutoShooting(ShootingTarget.Speaker, 36.0, 3200.0));
        
        // Calibration commands
        ShooterCalibrate.and(operator.povUp()).whileTrue(new ShooterAngleVelMove(2.0)); 
        ShooterCalibrate.and(operator.povDown()).whileTrue(new ShooterAngleVelMove(-2.0));
        ShooterCalibrate.and(operator.povLeft()).onTrue(
            new InstantCommand( () -> { shooter.setExtensionPosition(0.0); } )); 
        ClimberCalibrate.and(operator.povUp()).whileTrue(new ClimberVelocity(Climber.ClimbCalibrateVel));
        ClimberCalibrate.and(operator.povDown()).whileTrue(new ClimberVelocity(-Climber.ClimbCalibrateVel));
        ClimberCalibrate.and(operator.povLeft()).onTrue(
            new InstantCommand( ()-> {climber.setClimberPos(0.0); } ));
    }
}
