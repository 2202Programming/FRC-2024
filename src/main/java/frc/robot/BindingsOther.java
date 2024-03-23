package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.PDPMonitorCmd;
import frc.robot.commands.Climber.Climb;
import frc.robot.commands.Climber.ClimberVelocity;
import frc.robot.commands.Intake.AngleCalibration;
import frc.robot.commands.Intake.EjectNote;
import frc.robot.commands.Intake.InIntake;
import frc.robot.commands.Intake.IntakeSequence;
import frc.robot.commands.Intake.IntakeSwap;
import frc.robot.commands.Intake.MoveToAnglePos;
import frc.robot.commands.Intake.TestIntake;
import frc.robot.commands.Intake.TestIntakeAngle;
import frc.robot.commands.Shooter.CalibrateZero;
import frc.robot.commands.Shooter.PneumaticsSequence;
import frc.robot.commands.Shooter.RPMShooter;
import frc.robot.commands.Shooter.ShooterAngleSetPos;
import frc.robot.commands.Shooter.ShooterAngleVelMove;
import frc.robot.commands.Shooter.ShooterSequence;
import frc.robot.commands.Shooter.ShooterServoSequence;
import frc.robot.commands.Shooter.SpeakerShooter;
import frc.robot.commands.Shooter.TestShoot;
import frc.robot.commands.Swerve.AllianceAwareGyroReset;
import frc.robot.commands.Swerve.FaceToTag;
import frc.robot.commands.Swerve.RobotCentricDrive;
import frc.robot.commands.Swerve.RotateTo;
import frc.robot.commands.auto.AutoShooting;
import frc.robot.commands.auto.TurnFaceShootAuto;
import frc.robot.commands.auto.AutoShooting.ShootingTarget;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve.SwerveDrivetrain;
import frc.robot.subsystems.hid.HID_Xbox_Subsystem;

/*
 * Bindings here for testing, 
 */
public class BindingsOther {

    public static void ConfigureOther(HID_Xbox_Subsystem dc) {
        DriverBinding(dc);
        OperatorBindings(dc);
    }

    static void DriverBinding(HID_Xbox_Subsystem dc) {
        var driver = dc.Driver();
        var bindings = RobotContainer.bindings;
        var drivetrain = RobotContainer.getSubsystem(SwerveDrivetrain.class);
        var intake = RobotContainer.getSubsystem(Intake.class);

        switch (bindings) {

            case DriveTest:
                driver.leftBumper().whileTrue(new RobotCentricDrive(drivetrain, dc));
                driver.b().onTrue(new AllianceAwareGyroReset(false));

                // This appears to break if initial pose is too close to path start pose
                // (zero-length path?)
                driver.x().onTrue(new SequentialCommandGroup(
                        new InstantCommand(drivetrain::printPose),
                        AutoBuilder.pathfindThenFollowPath(PathPlannerPath.fromPathFile("blue1"),
                                new PathConstraints(3.0, 3.0,
                                        Units.degreesToRadians(540),
                                        Units.degreesToRadians(720))),
                        new InstantCommand(drivetrain::printPose)));

                driver.b().onTrue(new SequentialCommandGroup(
                        new InstantCommand(drivetrain::printPose),
                        AutoBuilder.pathfindThenFollowPath(PathPlannerPath.fromPathFile("red1"),
                                new PathConstraints(3.0, 3.0,
                                        Units.degreesToRadians(540),
                                        Units.degreesToRadians(720))),
                        new InstantCommand(drivetrain::printPose)));

                // Start any watcher commands
                new PDPMonitorCmd(); // auto scheduled, runs when disabled
                driver.leftTrigger().onTrue(new ShooterSequence(true, 1200.0));
                // This appears to break if initial pose is too close to path start pose
                // (zero-length path?)
                driver.a().onTrue(new SequentialCommandGroup(
                        new InstantCommand(drivetrain::printPose),
                        AutoBuilder.pathfindThenFollowPath(PathPlannerPath.fromPathFile("test_1m"),
                                new PathConstraints(3.0, 3.0, Units.degreesToRadians(540),
                                        Units.degreesToRadians(720))),
                        new InstantCommand(drivetrain::printPose)));

                driver.x().onTrue(new SequentialCommandGroup(
                        new InstantCommand(drivetrain::printPose),
                        AutoBuilder.pathfindToPose(new Pose2d(new Translation2d(1.73, 5.38), new Rotation2d(0.0)),
                                new PathConstraints(3.0, 3.0, Units.degreesToRadians(540),
                                        Units.degreesToRadians(720))),
                        new InstantCommand(drivetrain::printPose)));
                break;

            // i dont like that test commands and bindings are in here but we need them ig
            // --er
            case IntakeTesting:
                
                /*
                driver.rightBumper().whileTrue(new IntakeSequence(true));
                driver.povUp().onTrue(new ShooterSequence(true, 2000.0));
                driver.povRight().onTrue(new ShooterSequence(true, 1200.0));
                driver.povDown().whileTrue(new ShooterSequence(3200.0)); // RPM
                driver.leftBumper().whileTrue(new PneumaticsSequence());
                driver.x().whileTrue(new AngleCalibration(5.0));
                driver.y().whileTrue(new AngleCalibration(-5.0));
                // driver.leftBumper().whileTrue(new IntakeCalibrateForwardPos());
                driver.b().whileTrue(new TestIntake(0.35)); // % speed
                // driver.leftBumper().whileTrue(new TransferTest(30.0));
                driver.rightTrigger().onTrue(new MoveToAnglePos(Intake.TravelUp, Intake.TravelUp));
                driver.leftTrigger().onTrue(new MoveToAnglePos(Intake.TravelDown, Intake.TravelDown));
                // driver.rightTrigger().onTrue(new AnglePos(50.0));
                // driver.a().onTrue(new CalibratePos(0.0));
                */
                driver.rightBumper().onTrue(new InstantCommand( ()-> { intake.setIntakeSpeed(0.0);}));
                driver.a().onTrue(new InstantCommand( ()-> { intake.setIntakeSpeed(2.0);}));
                driver.b().onTrue(new InstantCommand( ()-> { 
                    intake.setIntakeSpeed(-2.0);}));
                driver.x().onTrue(new InstantCommand( ()-> { 
                    intake.setAngleSetpoint(105.0);}));
                driver.y().onTrue(new InstantCommand( ()-> { 
                    intake.setAngleSetpoint(0.0);}));
                break;

            case auto_shooter_test:
                driver.y().onTrue(new AllianceAwareGyroReset(true));
                driver.leftBumper().whileTrue(new RobotCentricDrive(drivetrain, dc));
                // I KNOW THIS IS BAD BUT I DONT REALLY CARE IF IT WORKS
                driver.povUp().onTrue(new ShooterServoSequence(28.52, 3000.0));
                driver.povLeft().onTrue(new ShooterServoSequence(46.0, 3000.0));

                // driver.povUp().onTrue(new SpeakerShooter( 3500.0));
                // driver.povLeft().onTrue(new SpeakerShooter( 3250.0));
                driver.povRight().onTrue(new SpeakerShooter(3000.0));
                driver.povDown().onTrue(new SpeakerShooter(2750.0));
                driver.a().onTrue(new FaceToTag(4.0));
                driver.b().onTrue(new RotateTo());
                driver.x().onTrue(new TurnFaceShootAuto(4));
                driver.rightBumper().onTrue(new SpeakerShooter(1750.0));
                break;

            case new_bot_test:
                driver.leftBumper().whileTrue(new RobotCentricDrive(drivetrain, dc));
                driver.y().onTrue(new AllianceAwareGyroReset(false));
                break;

            default:
                break;
        }
    }

    static void OperatorBindings(HID_Xbox_Subsystem dc) {
        var operator = dc.Operator();
        var bindings = RobotContainer.bindings;

        switch (bindings) {
            case Competition:
                BindingsCompetition.OperatorBindings(dc);
                break;

            case auto_shooter_test:
                operator.rightTrigger().whileTrue(new ShooterAngleVelMove(4.0));
                operator.leftBumper().whileTrue(new ShooterAngleVelMove(-4.0));
                operator.a().onTrue(new IntakeSequence(false));
                operator.x().onTrue(new AngleCalibration(-25.0));
                operator.b().onTrue(new AutoShooting(ShootingTarget.Speaker));
                operator.y().whileTrue(new EjectNote());
                operator.povUp().onTrue(new ShooterAngleSetPos(45));
                operator.rightBumper().onTrue(new ShooterAngleSetPos(28.6));
                operator.povDown().onTrue(new ShooterAngleSetPos(30));
                operator.povLeft().onTrue(new CalibrateZero());
                break;

            case Shooter_test:
                var shooter = RobotContainer.getSubsystem(Shooter.class);
                if (shooter != null) {
                    shooter.setDefaultCommand(new RPMShooter());
                }
                break;

            case IntakeTesting:
            /*
                operator.a().onTrue(new IntakeSequence(true)); // works for both modes
                operator.b().onTrue(new MoveToAnglePos(Intake.DownPos, 60.0));
                operator.x().onTrue(new InIntake(true));
                operator.y().whileTrue(new TestIntake(0.5)); // %

                operator.povDown().onTrue(new AngleCalibration(15.0)); // good for alpha
                operator.povUp().onTrue(new AngleCalibration(-15.0)); // not needed, calibrate with up

                operator.povRight().onTrue(new IntakeSwap());
                operator.povLeft().whileTrue(new EjectNote());

                // operator.rightBumper().whileTrue(new InIntake()); //works ---> seq for stay
                // in intake
                // operator.leftTrigger().whileTrue(new InAmp()); //works ---> into amp seq
                // operator.povRight().whileTrue(new IntakeTest(0.35));

                operator.rightBumper().onTrue(new ShooterSequence(true, 2000.0)); // speaker close
                operator.leftTrigger().onTrue(new ShooterSequence(true, 800.0)); // amp - NO WORK RN
                operator.rightTrigger().onTrue(new ShooterSequence(3500.0)); // speaker far - NO WORK RN
                */
                break;

            case new_bot_test:
                // // INTAKE & TRANSFER
                operator.x().whileTrue(new TestShoot(3000.0));
                operator.a().whileTrue(new TestIntake(140.0));
                // operator.a().onTrue(new InIntake(true)); // works for both modes
                // // operator.b().onTrue(new MoveToAnglePos(Intake.DownPos, 60.0));
                // // operator.x().onTrue(new InIntake(true));
                // operator.x().onTrue(new AngleCalibration(-25.0));
                // // operator.leftTrigger().whileTrue(new TestTransfer(35.0));
                // // operator.y().whileTrue(new TestIntake(0.5)); // %
                // // operator.povDown().onTrue(new MoveToAnglePos(Intake.DownPos,
                // // Intake.TravelDown)); // good for alpha
                // // operator.povUp().onTrue(new MoveToAnglePos(Intake.UpPos, Intake.TravelUp));
                // // // not needed, calibrate with  up
                // // operator.povRight().onTrue(new IntakeSwap());
                // // operator.leftBumper().whileTrue(new TestIntakeAngle(-25.0));
                // operator.y().whileTrue(new TestIntakeAngle(25.0));
                // // operator.povRight().onTrue(new MoveToAnglePos(9.63, 10.0));
                // // operator.povLeft().whileTrue(new EjectNote());
                // // operator.leftBumper().onTrue(new CalibratePos(0.0));
                // operator.povLeft().onTrue(new TestShoot(1500.0));
                // operator.rightTrigger().whileTrue(new TestShoot(1500.0));
                // operator.rightBumper().whileTrue(new TestShoot(3500.0));
                // SHOOTER
                // operator.rightTrigger().whileTrue(new ShootTs(500.0));
                // operator.povDown().whileTrue(new ShooterAngleVelMove(-2.0));
                // operator.povUp().whileTrue(new ShooterAngleVelMove(2.0));
                // operator.rightTrigger().whileTrue(new ShooterAngleVelMove(4.0));
                // operator.leftBumper().whileTrue(new ShooterAngleVelMove(-4.0));
                // // operator.rightBumper().onTrue(new ShooterServoSequence(30.0, 2000.0));
                // // //speaker close /
                // operator.leftTrigger().onTrue(new ShooterAngleSetPos(30.0));
                // operator.rightBumper().onTrue(new ShooterAngleSetPos(45.0));
                // operator.rightTrigger().whileTrue(new TestIntake(-1.0));
                // operator.povUp().whileTrue(new ClimberVelocity(10.0));
                // operator.povDown().whileTrue(new ClimberVelocity(-10.0));
                // operator.povLeft().onTrue(new Climb(28.0));
                // operator.povRight().onTrue(new Climb(-2.5));
                // operator.leftBumper().onTrue(new Climb(-0.0));
                // operator.povDown().whileTrue(new TestIntakeAngle(15.0));
                // operator.povUp().whileTrue(new TestIntakeAngle(-15.0));
                // operator.leftBumper(

                // ).whileTrue(new TestIntake(0.5));

                // operator.rightBumper().whileTrue(new InIntake()); //works ---> seq for stay
                // in intake
                // operator.leftTrigger().whileTrue(new InAmp()); //works ---> into amp seq
                // operator.povRight().whileTrue(new IntakeTest(0.35));
                break;
            default:
                break;
        }
    }

}
