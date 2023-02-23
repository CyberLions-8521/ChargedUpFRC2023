// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.JoystickArm;
import frc.robot.subsystems.ArmAndJoint;
//import frc.robot.commands.JoystickArm;
//import frc.robot.subsystems.ArmAndJoint;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.JoystickDriving;
import frc.robot.commands.tankDrive;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPRamseteCommand;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  //The robot's subsystems and commands are defined here...
  //SUBSYSTEMS
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final ArmAndJoint m_armAndJoint = new ArmAndJoint();
  private final JoystickArm m_joystickArm = new JoystickArm(m_armAndJoint);
  private final Drivebase m_drivebase = new Drivebase();
  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);
  private final SendableChooser<Command> m_chooser = new SendableChooser<>();

  public final JoystickDriving m_joystickDriving = new JoystickDriving(m_drivebase);
  public final tankDrive m_tankDrive = new  tankDrive(m_drivebase);

  //List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("New Path", new PathConstraints(4, 3));

  // HashMap<String, Command> eventMap = new HashMap<>();
  // eventMap.put("marker1", new PrintCommand("Passed marker 1"));
  // eventMap.put("intakeDown", new IntakeDown());

  
  //public final JoystickArm m_joystickArm = new JoystickArm(m_armAndJoint);

  PathPlannerTrajectory examplePath = PathPlanner.loadPath("bird", new PathConstraints(Constants.AutoConstants.kMaxSpeedMetersPerSecond, Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared));

  // ArrayList<PathPlannerTrajectory> pathGroup2 = PathPlanner.loadPathGroup(
  //   "Example Path Group", 
  //   new PathConstraints(4, 3), 
  //   new PathConstraints(2, 2), 
  //   new PathConstraints(3, 3));

  // Replace with CommandPS4Controller or CommandJoystick if needed
  public final static CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

      public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {
        return new SequentialCommandGroup(
            new InstantCommand(() -> {
              // Reset odometry for the first path you run during auto
              if(isFirstPath){
                  m_drivebase.resetOdometry(traj.getInitialPose());
              }
            }),
            new PPRamseteCommand(
                traj, 
                m_drivebase::getPose, // Pose supplier
                new RamseteController(),
                new SimpleMotorFeedforward(Constants.DriveConstants.ksVolts, Constants.DriveConstants.kvVoltSecondsPerMeter, Constants.DriveConstants.kaVoltSecondsSquaredPerMeter),
                m_drivebase.getKinematics(), // DifferentialDriveKinematics
                m_drivebase::getWheelSpeeds, // DifferentialDriveWheelSpeeds supplier
                new PIDController(Constants.DriveConstants.kPDriveVel, 0, Constants.DriveConstants.kIDriveVel, Constants.DriveConstants.kDDriveVel), // Left controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
                new PIDController(Constants.DriveConstants.kPDriveVel, 0, Constants.DriveConstants.kIDriveVel, Constants.DriveConstants.kDDriveVel), // Right controller (usually the same values as left controller)
                m_drivebase::tankDriveVolts, // Voltage biconsumer
                true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
                m_drivebase // Requires this drive subsystem
            )
        );
      }
    
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    //m_drivebase.setDefaultCommand(m_joystickDriving);
    m_chooser.setDefaultOption("Simple Auto", null);
    m_chooser.addOption("Complex Auto", null);
    SmartDashboard.putData(m_chooser);
    //m_drivebase.setDefaultCommand(m_tankDrive);
    m_armAndJoint.setDefaultCommand(m_joystickArm);
    configureBindings();
  }


  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    //m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());

    /*m_driverController.b().toggleOnTrue(m_armAndJoint.PIDArmAndJoint(0, 30));
    m_driverController.a().toggleOnTrue(m_armAndJoint.PIDArmAndJoint(1,25));
    m_driverController.a().toggleOnTrue(m_armAndJoint.PIDArmAndJoint(4,0));
    m_driverController.a().toggleOnTrue(m_armAndJoint.PIDArmAndJoint(2,-10));*/
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    //return m_armAndJoint.PIDArmAndJoint(3, 2);
    return followTrajectoryCommand(examplePath, true);
    //return m_chooser.getSelected();
    //return m_autoCommand;

  }
}
