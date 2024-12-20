// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//import com.pathplanner.lib.auto.NamedCommands;
//import com.pathplanner.lib.commands.PathPlannerAuto;

// import java.util.List;

//import java.util.List;

// import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.trajectory.Trajectory;
// import edu.wpi.first.math.trajectory.TrajectoryConfig;
// import edu.wpi.first.math.trajectory.TrajectoryGenerator;
//import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
// import frc.robot.Constants.AutoConstants;
// import frc.robot.Constants.DriveConstants;
// import frc.robot.Constants.AutoConstants;
// import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.ZeroHeading;
// import frc.robot.commands.AmpPoint;
// import frc.robot.commands.MoveShooter;
import frc.robot.commands.ClimbStop;
import frc.robot.commands.ClimberRelease;
import frc.robot.commands.DropObject;
import frc.robot.commands.IntakeRun;
// import frc.robot.commands.IntakeStop;
import frc.robot.commands.LeftClimb;
import frc.robot.commands.LeftClimbUp;
// import frc.robot.commands.PivotDown;
// import frc.robot.commands.PivotStop;
import frc.robot.commands.PrepareNote;
import frc.robot.commands.RightClimb;
import frc.robot.commands.RightClimbUp;
import frc.robot.commands.RotateToTarget;
import frc.robot.commands.ShootAmp;
import frc.robot.commands.ShootSpeaker;
import frc.robot.commands.ShooterStop;
import frc.robot.commands.Shooterspeed1;
import frc.robot.commands.Shooterspeed2;
import frc.robot.commands.Shooterspeed3;
// import frc.robot.commands.ArmAdjust;
// import frc.robot.commands.ArmAdjustDown;
// import frc.robot.commands.ArmDown;
// import frc.robot.commands.ArmMidRow;
// //import frc.robot.commands.ArmPick;
// import frc.robot.commands.ArmTopRow;
// //import frc.robot.commands.ArmTravel;
import frc.robot.commands.AutonomousC;
// import frc.robot.commands.DropObject;
// import frc.robot.commands.IntakeCone;
// import frc.robot.commands.IntakeCube;
// import frc.robot.commands.IntakeIdle;
import frc.robot.commands.IntakeStop;
// import frc.robot.commands.TOFTester;
// import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Intake;
// import frc.robot.subsystems.Joysticks;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
// import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
//import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.Timer;



/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  public final static DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final ZeroHeading zeroHeading;
  private final Intake intake;
  private final Shooter shooter;
  private final Timer timer;
  private final Climber climber;
  private final IntakeRun intakeRun; 
  private final IntakeStop intakeStop;
  private final ShootSpeaker shootSpeaker;
  private final ShooterStop shooterStop;
  private final DropObject dropObject;
  private final PrepareNote prepareNote;
  private final ClimberRelease climberRelease;
  private final LeftClimb leftClimb;
  private final RightClimb rightClimb;
  private final ClimbStop climbStop;
  private final LeftClimbUp leftClimbUp;
  private final RightClimbUp rightClimbUp;
  private final Pivot pivot;
//   private final AmpPoint ampPoint;
//   private final PivotDown pivotDown;
//   private final MoveShooter moveShooter;
//   private final PivotStop stopShooter;
  private final ShootAmp shootAmp;
//   private final Joysticks joysticks;
  // private final DropObject dropObject;
  private final AutonomousC autonomousC;
  private final Shooterspeed1 shooterspeed1;
  private final Shooterspeed2 shooterspeed2;
  private final Shooterspeed3 shooterspeed3;
  private final RotateToTarget rotateToTarget;
  
  

  // The driver's controller
    public static XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
    public static XboxController m_operatorController = new XboxController(OIConstants.kOperatorControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // CameraServer.startAutomaticCapture(0);
  
        intake = new Intake();
        timer = new Timer();
        zeroHeading = new ZeroHeading(m_robotDrive);
        zeroHeading.addRequirements(m_robotDrive);
        shooter = new Shooter();
        climber = new Climber();
        pivot = new Pivot();
        // joysticks = new Joysticks();
        timer.reset();
        // NamedCommands.registerCommand("ShootSpeaker", new ShootSpeaker(shooter));
        // NamedCommands.registerCommand("PrepareNote", new PrepareNote(intake));
        // NamedCommands.registerCommand("IntakeRun", new IntakeRun(intake, m_robotDrive));
        // NamedCommands.registerCommand("AutonomousC", new AutonomousC(intake, shooter));
        intakeRun = new IntakeRun(intake, m_robotDrive);
        intakeRun.addRequirements(intake, m_robotDrive);
        intakeStop = new IntakeStop(intake);
        intakeStop.addRequirements(intake);
        shootSpeaker = new ShootSpeaker(shooter);
        shootSpeaker.addRequirements(shooter, intake);
        shooterStop = new ShooterStop(shooter);
        shooterStop.addRequirements(shooter);
        dropObject = new DropObject(intake);
        dropObject.addRequirements(intake);
        prepareNote = new PrepareNote(intake);
        prepareNote.addRequirements(intake);
        climberRelease = new ClimberRelease(climber);
        climberRelease.addRequirements(climber);
        leftClimb = new LeftClimb(climber);
        leftClimb.addRequirements(climber);
        rightClimb = new RightClimb(climber);
        rightClimb.addRequirements(climber);
        leftClimbUp = new LeftClimbUp(climber);
        leftClimbUp.addRequirements(climber);
        rightClimbUp = new RightClimbUp(climber);
        rightClimbUp.addRequirements(climber);
        climbStop = new ClimbStop(climber);
        climbStop.addRequirements(climber);
        // ampPoint = new AmpPoint(pivot);
        // ampPoint.addRequirements(pivot);
        // pivotDown = new PivotDown(pivot);
        // pivotDown.addRequirements(pivot);
        // moveShooter = new MoveShooter(pivot, 0.0);
        // moveShooter.addRequirements(pivot);
        // stopShooter = new PivotStop(pivot);
        // stopShooter.addRequirements(pivot);
        shootAmp = new ShootAmp(shooter);
        shootAmp.addRequirements(shooter);
        shooterspeed1 = new Shooterspeed1(shooter);
        shooterspeed1.addRequirements(shooter);
        shooterspeed2 = new Shooterspeed2(shooter);
        shooterspeed2.addRequirements(shooter);
        shooterspeed3 = new Shooterspeed3(shooter);
        shooterspeed3.addRequirements(shooter);
        // dropObject = new DropObject(intake);
        // dropObject.addRequirements(intake);
        autonomousC = new AutonomousC(intake, shooter);
        autonomousC.addRequirements(intake, shooter);
        rotateToTarget = new RotateToTarget(m_robotDrive);
        rotateToTarget.addRequirements(m_robotDrive);

    // Configure the button bindings
    configureButtonBindings();

    


    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        Commands.run(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true, true),
            m_robotDrive));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    // new JoystickButton(m_driverController, XboxController.Button.kX.value)
    //     .whileTrue(new RunCommand(
    //         () -> m_robotDrive.setX(),
    //         m_robotDrive));

    new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value)
        .whileTrue(new ShootSpeaker(shooter))
        .onFalse(new ShooterStop(shooter));

    new JoystickButton(m_driverController, XboxController.Button.kStart.value)
        .whileTrue(new RotateToTarget(m_robotDrive));

    new JoystickButton(m_driverController, XboxController.Button.kBack.value)
        .whileTrue(new ShootAmp(shooter))
        .onFalse(new ShooterStop(shooter));

    new JoystickButton(m_operatorController, XboxController.Button.kX.value)
        .whileTrue(new IntakeRun(intake, m_robotDrive))
        .onFalse(new IntakeStop(intake));


    new JoystickButton(m_operatorController, XboxController.Button.kB.value)
        .whileTrue(new DropObject(intake))
        .onFalse(new IntakeStop(intake));

    new JoystickButton(m_driverController, XboxController.Button.kX.value)
        .whileTrue(new ZeroHeading(m_robotDrive));

    new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value)
        .whileTrue(new PrepareNote(intake))
        .onFalse(new IntakeStop(intake));

    // new JoystickButton(m_operatorController, XboxController.Button.kX.value) 
    //     .onTrue(new ClimberRelease(climber));

    new JoystickButton(m_operatorController, XboxController.Button.kLeftBumper.value)
        .whileTrue(new LeftClimb(climber))
        .onFalse(new ClimbStop(climber));

    new JoystickButton(m_operatorController, XboxController.Button.kStart.value)
        .whileTrue(new RightClimb(climber))
        .onFalse(new ClimbStop(climber));

    new JoystickButton(m_operatorController, XboxController.Button.kBack.value)
        .whileTrue(new LeftClimbUp(climber))
        .onFalse(new ClimbStop(climber));

    new JoystickButton(m_operatorController, XboxController.Button.kRightBumper.value)
        .whileTrue(new RightClimbUp(climber))
        .onFalse(new ClimbStop(climber));
    //.onTrue(new AmpPoint(pivot));

    new JoystickButton(m_driverController, XboxController.Button.kY.value)
        .whileTrue(new Shooterspeed3(shooter))
        .onFalse(new ShooterStop(shooter));

    new JoystickButton(m_driverController, XboxController.Button.kB.value)
        .whileTrue(new Shooterspeed2(shooter))
        .onFalse(new ShooterStop(shooter));

    new JoystickButton(m_driverController, XboxController.Button.kA.value)
        .whileTrue(new Shooterspeed1(shooter))
        .onFalse(new ShooterStop(shooter));

    // new JoystickButton(m_driverController, XboxController.Button.kA.value)
    //     .onTrue(new MoveShooter(pivot, 0.95));
        
    // new JoystickButton(m_driverController, XboxController.Button.kY.value)
    //     .onTrue(new MoveShooter(pivot, 0.01));

    // new JoystickButton(m_driverController, XboxController.Button.kB.value)
    //     .onTrue(new PivotStop(pivot));
  }
  /*   
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
public Command getAutonomousCommand() {
  
// Create config for trajectory
    // TrajectoryConfig config = new TrajectoryConfig(
    //     AutoConstants.kMaxSpeedMetersPerSecond,
    //     AutoConstants.kMaxAccelerationMetersPerSecondSquared)
    //     // Add kinematics to ensure max speed is actually obeyed
    //     .setKinematics(DriveConstants.kDriveKinematics);

    // // An example trajectory to follow. All units in meters.
    // Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
    //     // Start at the origin facing the +X direction
    //     new Pose2d(0, 0, new Rotation2d(0)),
    //     // Pass through these two interior waypoints, making an 's' curve path
    //     List.of(new Translation2d(.33, 0), new Translation2d(.66, 0)),
    //     // End 3 meters straight ahead of where we started, facing forward
    //     new Pose2d(1, 0, new Rotation2d(2)),
    //     config);

    // var thetaController = new ProfiledPIDController(
    //     AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    // thetaController.enableContinuousInput(-Math.PI, Math.PI);

    // SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
    //     exampleTrajectory,
    //     m_robotDrive::getPose, // Functional interface to feed supplier
    //     DriveConstants.kDriveKinematics,

    //     // Position controllers
    //     new PIDController(AutoConstants.kPXController, 0, 0),
    //     new PIDController(AutoConstants.kPYController, 0, 0),
    //     thetaController,
    //     m_robotDrive::setModuleStates,
    //     m_robotDrive);

    // // Reset odometry to the starting pose of the trajectory.
    // m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // // Run path following command, then stop at the end.
    // return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false, false));
 //   return new PathPlannerAuto("Shoot Test Auto");
 return autonomousC;
}

  
}
