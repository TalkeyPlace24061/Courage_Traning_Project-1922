// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Neo;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import static edu.wpi.first.units.Units.*;
import frc.robot.subsystems.ServoMotor;
import frc.robot.subsystems.Telemetry;
import frc.robot.subsystems.TimeOfFlight;
import frc.robot.subsystems.ColorDecector;
import frc.robot.commands.usingTheSubsystems;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.commands.TunerConstants;
import frc.robot.commands.autoDrive;
import frc.robot.subsystems.LimeLight;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final ServoMotor m_ServoMotor = new ServoMotor();
  private final LED m_LED = new LED(m_ServoMotor);
  private final Neo m_Neo = new Neo();
  private final TimeOfFlight m_TOF = new TimeOfFlight();
  private final ColorDecector m_CD = new ColorDecector();
  private final usingTheSubsystems m_UsingTheSubsystems = new usingTheSubsystems(m_ServoMotor, m_LED, m_Neo, m_TOF,
      m_CD);
  private final CommandSwerveDrivetrain m_SwerveDrivetrain = TunerConstants.createDrivetrain();
  private final LimeLight m_LimeLight = new LimeLight();
  private final autoDrive m_AutoDrive = new autoDrive(m_LimeLight, m_SwerveDrivetrain);
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController = new CommandXboxController(
      OperatorConstants.kDriverControllerPort);

  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max
                                                                                    // angular velocity

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  private final Telemetry logger = new Telemetry(MaxSpeed);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    // m_ServoMotor.setDefaultCommand(m_UsingTheSubsystems);
    SmartDashboard.putData("Auto Drive", m_AutoDrive);
    SmartDashboard.putData("limeLight", m_LimeLight);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // new Trigger(m_exampleSubsystem::exampleCondition)
    // .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is
    // pressed,
    // cancelling on release.
    m_driverController.y().whileTrue(m_AutoDrive);

    m_SwerveDrivetrain.setDefaultCommand(
        // m_SwerveDrivetrain will execute this command periodically
        m_SwerveDrivetrain.applyRequest(() -> drive.withVelocityX(-m_driverController.getLeftY() * MaxSpeed) // Drive
                                                                                                             // forward
                                                                                                             // with
                                                                                                             // negative
                                                                                                             // Y
                                                                                                             // (forward)
            .withVelocityY(-m_driverController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-m_driverController.getRightX() * MaxAngularRate) // Drive counterclockwise with
                                                                                  // negative X (left)
        ));

    // Idle while the robot is disabled. This ensures the configured
    // neutral mode is applied to the drive motors while disabled.
    final var idle = new SwerveRequest.Idle();
    RobotModeTriggers.disabled().whileTrue(
        m_SwerveDrivetrain.applyRequest(() -> idle).ignoringDisable(true));

    m_driverController.a().whileTrue(m_SwerveDrivetrain.applyRequest(() -> brake));
    m_driverController.b().whileTrue(m_SwerveDrivetrain.applyRequest(() -> point
        .withModuleDirection(new Rotation2d(-m_driverController.getLeftY(), -m_driverController.getLeftX()))));

    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log.
    m_driverController.back().and(m_driverController.y())
        .whileTrue(m_SwerveDrivetrain.sysIdDynamic(Direction.kForward));
    m_driverController.back().and(m_driverController.x())
        .whileTrue(m_SwerveDrivetrain.sysIdDynamic(Direction.kReverse));
    m_driverController.start().and(m_driverController.y())
        .whileTrue(m_SwerveDrivetrain.sysIdQuasistatic(Direction.kForward));
    m_driverController.start().and(m_driverController.x())
        .whileTrue(m_SwerveDrivetrain.sysIdQuasistatic(Direction.kReverse));

    // reset the field-centric heading on left bumper press
    m_driverController.leftBumper().onTrue(m_SwerveDrivetrain.runOnce(() -> m_SwerveDrivetrain.seedFieldCentric()));

    m_SwerveDrivetrain.registerTelemetry(logger::telemeterize);

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);

  }
}
