// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveRequest;

//import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
//import edu.wpi.first.wpilibj2.command.InstantCommand;
//import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
//import pabeles.concurrency.IntOperatorTask.Max;
//import static edu.wpi.first.units.Units.*;

//import com.ctre.phoenix6.CANBus;
//import com.ctre.phoenix6.StatusCode;
//import com.ctre.phoenix6.configs.TalonFXConfiguration;
//import com.ctre.phoenix6.controls.Follower;
//import com.ctre.phoenix6.controls.NeutralOut;
//import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
//import com.ctre.phoenix6.hardware.TalonFX;
//import com.ctre.phoenix6.signals.MotorAlignmentValue;

//import edu.wpi.first.wpilibj.TimedRobot;

import com.revrobotics.spark.config.SparkMaxConfig;
//import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
//import com.ctre.phoenix6.controls.VelocityVoltage;
import com.revrobotics.spark.SparkClosedLoopController;
//import edu.wpi.first.math.controller.ArmFeedforward;
//import com.pathplanner.lib.path.PathPlannerPath;
//import com.pathplanner.lib.auto.AutoBuilder;
//import com.pathplanner.lib.path.PathConstraints;
//import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import com.ctre.phoenix6.controls.Follower;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.VecBuilder;   

public class RobotContainer {
    public boolean armStatus = false;
    public boolean limelightStatus = false;
    public boolean intakeStatus = false;
    public boolean shooterStatus = false;
    private double targetRPS = 60;;
    private Command ShootlaCopy;
    private TalonFX intakeMotor1 = new TalonFX(30, "Drivetrain");
    private TalonFX intakeMotor2 = new TalonFX(31, "Drivetrain");
    private SparkMax armMotor = new SparkMax(36, MotorType.kBrushless);
    private SparkMax FeederMotor = new SparkMax(32, MotorType.kBrushless);
    private TalonFX shooterMotor1 = new TalonFX(33, "Drivetrain");
    private TalonFX shooterMotor2 = new TalonFX(34, "Drivetrain");
    private TalonFX shooterFeeder = new TalonFX(35, "Drivetrain");
        
    private final double kP_Translation = 2; 
    private final double kP_Rotation = 2;
    private Pose2d kTargetPose = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0));
    private Pose2d kMainTargetPose = new Pose2d(-2.0, 4.0, Rotation2d.fromDegrees(0));
    private Pose2d kAutonomousStartPose = new Pose2d(-2.0, 4.0, Rotation2d.fromDegrees(0));
    private boolean startedOnRight = true; // true: sağda başlıyor, false: solda başlıyor
    private boolean drivingToPose = false;

    private double MaxSpeed = 0.5 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    private double kMaxSpeed = 0.2 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double kMaxAngularRate = RotationsPerSecond.of(0.35).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    
    private double lastPrintTime = 0.0;

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final SwerveRequest.FieldCentricFacingAngle driveFacing = new SwerveRequest.FieldCentricFacingAngle()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage).withMaxAbsRotationalRate(MaxAngularRate);

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandPS5Controller joystick = new CommandPS5Controller(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private final VelocityVoltage shooterVelocityRequest1 = new VelocityVoltage(0);
    //private final VelocityVoltage shooterVelocityRequest2 = new VelocityVoltage(0);
    private final VelocityVoltage shooterFeederVelocityRequest = new VelocityVoltage(0);

    public RobotContainer() {
        var pidmain = new com.ctre.phoenix6.configs.TalonFXConfiguration();
        pidmain.Slot0.kP = 0.65;
        pidmain.Slot0.kD = 0.065;

        pidmain.CurrentLimits.SupplyCurrentLimit = 35;
        pidmain.CurrentLimits.SupplyCurrentLimitEnable = true;
        pidmain.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        
        pidmain.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        intakeMotor2.getConfigurator().apply(pidmain);

        pidmain.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        shooterMotor1.getConfigurator().apply(pidmain);
        intakeMotor1.getConfigurator().apply(pidmain);
        
        pidmain.Slot0.kP = 0.8;
        pidmain.Slot0.kD = 0.08;
        shooterFeeder.getConfigurator().apply(pidmain);

        shooterMotor2.setControl(new Follower(shooterMotor1.getDeviceID(), MotorAlignmentValue.Aligned));

        SparkMaxConfig config = new SparkMaxConfig();
        config.smartCurrentLimit(35);
        config.inverted(false);
        config.closedLoop.pid(0.75, 0, 0.075);
        config.idleMode(IdleMode.kCoast);

        FeederMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkMaxConfig armConfig = new SparkMaxConfig();
        armConfig.smartCurrentLimit(40);
        armConfig.idleMode(IdleMode.kBrake);
        armConfig.inverted(false);
        armConfig.encoder.positionConversionFactor(360.0/80);

        armMotor.configure(armConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        armMotor.getEncoder().setPosition(0);
        
        configureBindings(); 
    
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            ).unless(() -> drivingToPose)
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
                drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        joystick.L1().onTrue(Commands.runOnce(() -> {
            if (intakeStatus) {
                intakeStatus = false;
                intakeMotor1.set(0.0);
                intakeMotor2.set(0);
            } else {
                intakeStatus = true;
                intakeMotor1.set(0.7);
                intakeMotor2.set(0.7);
            }
        }));//square -> ps5

        joystick.R1().onTrue(Commands.runOnce(() -> {
        if (shooterStatus) {
            shooterStatus = false;
            if(ShootlaCopy != null) ShootlaCopy.cancel();
            shooterFeeder.set(0);
            shooterMotor1.set(0);
            FeederMotor.set(0);
        } else {
            shooterStatus = true;
            targetRPS = ((double)57.0);
            ShootlaCopy = Shootla(targetRPS);
            ShootlaCopy.schedule();
        }
        }));//circle -> ps5

        joystick.triangle().onTrue(Commands.runOnce(() -> {
            if(drivingToPose)  drivingToPose = false;   
            else driveToPoseCommand().schedule();
        }));//triangle -> ps5

        joystick.povUp().whileTrue(Commands.run(() -> armMotor.set(0.1)))
                .onFalse(Commands.runOnce(() -> armMotor.set(0.0)));

        joystick.povDown().whileTrue(Commands.run(() -> armMotor.set(-0.3)))
                .onFalse(Commands.runOnce(() -> armMotor.set(0.0)));

        //joystick.povUp().onTrue(new RunCommand(()->SmartDashboard.putNumber("Shooter1Rpm", shooterMotor1.getEncoder().getVelocity())));
        joystick.circle().onTrue(Commands.runOnce(()->{kTargetPose = drivetrain.getState().Pose; System.out.println(kTargetPose);},drivetrain));
        joystick.square().onTrue(Commands.runOnce(()->{limelightStatus = !limelightStatus; System.err.println(limelightStatus);},drivetrain));
        joystick.povLeft().onTrue(Commands.runOnce(() -> {
            LimelightHelpers.SetRobotOrientation("limelight",
                drivetrain.getState().Pose.getRotation().getDegrees(), 0, 0, 0, 0, 0);
            LimelightHelpers.PoseEstimate mt2 =
                LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");

            if (mt2 != null && mt2.tagCount > 0) {
                drivetrain.resetPose(mt2.pose);
                System.out.printf(
                    "[Limelight] Pose RESET (MT2) -> X: %.2f m, Y: %.2f m, Yaw: %.2f°%n",
                    mt2.pose.getX(), mt2.pose.getY(), mt2.pose.getRotation().getDegrees()
                );
            } else {
                System.out.println("Tag Error.");
            }
        }));
        
        new Trigger(() -> shooterStatus)
            .whileTrue(drivetrain.applyRequest(() -> brake));
    
        drivetrain.registerTelemetry((state) -> {
            logger.telemeterize(state);
            System.out.println(drivetrain.getState().Pose);
            if(limelightStatus){
            LimelightHelpers.SetRobotOrientation(
                "limelight",
                state.Pose.getRotation().getDegrees(),
                0, 0, 0, 0, 0
            );
            LimelightHelpers.PoseEstimate mt2 =
                LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
            boolean rejectUpdate = false;
            if (Math.abs(state.Speeds.omegaRadiansPerSecond) > Math.toRadians(720)) {
                rejectUpdate = true;
            }
            if (mt2 == null || mt2.tagCount == 0) {
                rejectUpdate = true;
            }
            if (!rejectUpdate) {
                double xyStdDev = 0.5 * (1.0 + mt2.avgTagDist * 0.1) / Math.max(1, mt2.tagCount);
                drivetrain.setVisionMeasurementStdDevs(
                    VecBuilder.fill(xyStdDev, xyStdDev, 9999999)
                );
                drivetrain.addVisionMeasurement(mt2.pose, mt2.timestampSeconds);
                double now = Timer.getFPGATimestamp();
                if (now - lastPrintTime >= 1.0) {
                    lastPrintTime = now;
                    System.out.printf(
                        "[Limelight] -> X: %.2f m, Y: %.2f m, Yaw: %.2f°, TagCount: %d, AvgDist: %.2f m%n",
                        mt2.pose.getX(),
                        mt2.pose.getY(),
                        mt2.pose.getRotation().getDegrees(),
                        mt2.tagCount,
                        mt2.avgTagDist
                    );
                }
            }
        }});
    }

    public Command driveToPoseCommand() {
        return drivetrain.applyRequest(() -> {
            
            Pose2d currentPose = drivetrain.getState().Pose;

            double xError = kTargetPose.getX() - currentPose.getX();
            double yError = kTargetPose.getY() - currentPose.getY();
            
            double angleError = edu.wpi.first.math.MathUtil.inputModulus(
                kTargetPose.getRotation().getRadians() - currentPose.getRotation().getRadians(),
                -Math.PI, 
                Math.PI
            );

            double xVel = xError * kP_Translation;
            double yVel = yError * kP_Translation;
            double rotVel = angleError * kP_Rotation;

            if(xVel > 0) xVel = Math.min(xVel, kMaxSpeed);
            else xVel = Math.max(xVel, -kMaxSpeed);

            if(yVel > 0) yVel = Math.min(yVel, kMaxSpeed);
            else yVel = Math.max(yVel, -kMaxSpeed);
            
            if(rotVel > 0) rotVel = Math.min(rotVel, kMaxAngularRate);
            else rotVel = Math.max(rotVel, -kMaxAngularRate);

            return drive.withVelocityX(-xVel)
                        .withVelocityY(-yVel)
                        .withRotationalRate(-rotVel);
        })
        .beforeStarting(() -> drivingToPose = true)
        .until(() -> {
            Pose2d currentPose = drivetrain.getState().Pose;
            double dist = currentPose.getTranslation().getDistance(kTargetPose.getTranslation());
            double degError = Math.abs(currentPose.getRotation().minus(kTargetPose.getRotation()).getDegrees());
            
            return (!drivingToPose || (dist < 0.01 && degError < 3.0));
        })
        .finallyDo((interrupted) -> drivingToPose = false); 
    }

    public Command Shootla(double targetRPS) {
        return Commands.run(() -> {
            shooterMotor1.setControl(shooterVelocityRequest1.withVelocity(targetRPS));
            shooterFeeder.setControl(shooterFeederVelocityRequest.withVelocity(targetRPS*0.85));
            if (shooterMotor1.getVelocity().getValueAsDouble() >= targetRPS * 0.85 && shooterStatus) {
                FeederMotor.set(0.8);
            }
        })
        .until(() -> !shooterStatus)
        .finallyDo((interrupted) -> {
            shooterMotor1.set(0);
            shooterFeeder.set(0);
            FeederMotor.set(0);
        });
    }

    public void onTeleopInit() {
        limelightStatus = false;
        kTargetPose = kMainTargetPose;
        drivetrain.resetPose(new Pose2d(0,0,drivetrain.getState().Pose.getRotation()));
    }

    public Command getAutonomousCommand() {
    double mirror = startedOnRight ? 1.0 : -1.0;
    double firstTurnAngle = 90.0 * mirror; // sağda başladıysa +90 (sola dön), solda başladıysa -90 (sağa dön)

    double startX = kAutonomousStartPose.getX();
    double startY = kAutonomousStartPose.getY();
        
    return Commands.sequence(
        // pose reset
        Commands.runOnce(() -> drivetrain.resetPose(kAutonomousStartPose)),
        // düz git
        Commands.runOnce(() -> kTargetPose = new Pose2d(startX + 4.5, startY, Rotation2d.fromDegrees(0))),
        driveToPoseCommand(),
        // dön
        Commands.runOnce(() -> kTargetPose = new Pose2d(kTargetPose.getX(), kTargetPose.getY(), Rotation2d.fromDegrees(firstTurnAngle))),
        driveToPoseCommand(),
        // arm çalıştır
        Commands.run(() -> armMotor.set(0.3)).withTimeout(2.0),
        Commands.runOnce(() -> armMotor.set(0.0)),
        // intake çalıştır
        Commands.runOnce(() -> {
            intakeStatus = true;
            intakeMotor1.set(0.7);
            intakeMotor2.set(0.7);
        }),
        // yana git 
        Commands.runOnce(() -> kTargetPose = new Pose2d(startX + 4.5, startY + 3.0 * mirror, Rotation2d.fromDegrees(firstTurnAngle))),
        driveToPoseCommand(),
        // geri dön
        Commands.runOnce(() -> kTargetPose = new Pose2d(startX + 4.5, startY, Rotation2d.fromDegrees(0))),
        driveToPoseCommand(),
        // alttan geç
        Commands.runOnce(() -> kTargetPose = new Pose2d(startX - 2.0, startY, Rotation2d.fromDegrees(0))),
        driveToPoseCommand(),
        // shoot pozisyonuna 
        Commands.runOnce(() -> kTargetPose = kMainTargetPose),
        driveToPoseCommand(),
        // shootla
        Commands.runOnce(() -> {
            shooterStatus = true;
            targetRPS = 51.0;
            FeederMotor.set(-0.3);
            ShootlaCopy = Shootla(targetRPS);
            ShootlaCopy.schedule();
        }),
        Commands.waitSeconds(5.0),
        Commands.runOnce(() -> {
            shooterStatus = false;
            if (ShootlaCopy != null) ShootlaCopy.cancel();
            shooterFeeder.set(0);
            shooterMotor1.set(0);
            FeederMotor.set(0);
        })
    );
}


}
