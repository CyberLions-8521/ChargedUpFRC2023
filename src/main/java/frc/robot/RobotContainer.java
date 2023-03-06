// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Balancing;
//import frc.robot.commands.Balancing;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.JoystickArm;
import frc.robot.subsystems.ArmAndJoint;

import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.RamseteAutoBuilder;
import frc.robot.subsystems.Claw;
//import frc.robot.commands.JoystickArm;
//import frc.robot.subsystems.ArmAndJoint;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Limelight;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.JoystickDriving;
import frc.robot.commands.TurnToTarget;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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
import edu.wpi.first.wpilibj.XboxController;
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
  private final Claw m_claw = new Claw();
  private final ArmAndJoint m_armAndJoint = new ArmAndJoint();
  private final JoystickArm m_joystickArm = new JoystickArm(m_armAndJoint);
  private final Drivebase m_drivebase = new Drivebase();
  private final Limelight m_limelight = new Limelight();
  private final Balancing m_balance = new Balancing(m_drivebase);
  private final TurnToTarget m_turnToTarget = new TurnToTarget(m_limelight, m_drivebase, "april tag");
  public final JoystickDriving m_joystickDriving = new JoystickDriving(m_drivebase, m_limelight);
  private final SendableChooser<Command> m_chooser = new SendableChooser<>();
  PathPlannerTrajectory birb = PathPlanner.loadPath("emmy", new PathConstraints(Constants.AutoConstants.kMaxSpeedMetersPerSecond, Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared));
  // List<PathPlannerTrajectory> BluTopPos2NodesCharge = PathPlanner.loadPathGroup("BluTopPos2NodesCharge", new PathConstraints(Constants.AutoConstants.kMaxSpeedMetersPerSecond, Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared));
  // List<PathPlannerTrajectory> BluTopPos3Nodes = PathPlanner.loadPathGroup("BluTopPos3Nodes", new PathConstraints(Constants.AutoConstants.kMaxSpeedMetersPerSecond, Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared));
  // List<PathPlannerTrajectory> RedTopPos2NodesCharge = PathPlanner.loadPathGroup("RedTopPos2NodesCharge", new PathConstraints(Constants.AutoConstants.kMaxSpeedMetersPerSecond, Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared));
  // List<PathPlannerTrajectory> RedTopPos3Nodes = PathPlanner.loadPathGroup("RedTopPos3Nodes", new PathConstraints(Constants.AutoConstants.kMaxSpeedMetersPerSecond, Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared));
  
  // private Command auto1 = autoDrive.fullAuto(pathGroup);
  PathPlannerTrajectory examplePath = PathPlanner.loadPath("emmy", new PathConstraints(Constants.AutoConstants.kMaxSpeedMetersPerSecond, Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared));
  //PIDConstants bruh = new PIDConstants(DriveConstants.kPDriveVel, DriveConstants.kIDriveVel, DriveConstants.kDDriveVel);
  HashMap<String, Command> eventMap = new HashMap<>();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  public final static XboxController m_driverController_reg = new XboxController(1);
  public final static CommandXboxController m_driverController =
      new CommandXboxController(1);
/* 
  RamseteAutoBuilder autoDrive = new RamseteAutoBuilder(
    m_drivebase::getPose, // Pose supplier
    m_drivebase::resetOdometry,
    new RamseteController(),
    m_drivebase.getKinematics(), // DifferentialDriveKinematics
    new SimpleMotorFeedforward(Constants.DriveConstants.ksVolts, Constants.DriveConstants.kvVoltSecondsPerMeter, Constants.DriveConstants.kaVoltSecondsSquaredPerMeter),
    m_drivebase::getWheelSpeeds, // DifferentialDriveWheelSpeeds supplier
    bruh,
    m_drivebase::tankDriveVolts, // Voltage biconsumer
    eventMap,
    true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
    m_drivebase // Requires this drive subsystem
  );*/

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
                new RamseteController(2, 0.7),
                new SimpleMotorFeedforward(Constants.DriveConstants.ksVolts,
                 Constants.DriveConstants.kvVoltSecondsPerMeter, 
                 Constants.DriveConstants.kaVoltSecondsSquaredPerMeter),
                m_drivebase.getKinematics(), // DifferentialDriveKinematics
                //m_drivebase.m_kinematics,
                m_drivebase::getWheelSpeeds, // DifferentialDriveWheelSpeeds supplier
                new PIDController(Constants.DriveConstants.kPDriveVel, 0, 
                  Constants.DriveConstants.kDDriveVel), // Left controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
                new PIDController(Constants.DriveConstants.kPDriveVel, 0, 
                  Constants.DriveConstants.kDDriveVel), // Right controller (usually the same values as left controller)
                m_drivebase::tankDriveVolts, // Voltage biconsumer
                false, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
                m_drivebase // Requires this drive subsystem
            )
        );
      }
  //Command a = followTrajectoryCommand(examplePath,false);
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // eventMap.put("arm_down", m_armAndJoint.PIDArmAndJoint(0, 0));
    // eventMap.put("arm_up", m_armAndJoint.PIDArmAndJoint(0, 0));
    // eventMap.put("balance", new Balancing(m_drivebase));
    // eventMap.put("claw_grab", m_claw.Grab());
    // eventMap.put("claw_drop", m_claw.Release());
    m_drivebase.setDefaultCommand(m_joystickDriving);
    //m_chooser.setDefaultOption("Simple Auto", followTrajectoryCommand(birb, true));
    // m_chooser.addOption("BluTopPos2NodesCharge",  autoDrive.fullAuto(BluTopPos2NodesCharge));
    // m_chooser.addOption("BluTopPos3Nodes",  autoDrive.fullAuto(BluTopPos3Nodes));
    // m_chooser.addOption("RedTopPos3Nodes",  autoDrive.fullAuto(RedTopPos3Nodes));
    // m_chooser.addOption("RedTopPos2NodesCharge",  autoDrive.fullAuto(RedTopPos2NodesCharge));
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
    
    m_driverController.y().whileTrue(m_claw.Grab());
    m_driverController.b().whileTrue(m_claw.Release());
    m_driverController.a().whileTrue(new TurnToTarget(m_limelight, m_drivebase, "lol"));
    m_driverController.x().onTrue(m_armAndJoint.PIDArmAndJoint(0.72, 1));
   // m_driverController.x().onTrue(m_armAndJoint.moveToAngle(65));
   // m_driverController.x().onTrue(m_armAndJoint.moveArm(0.3));
    SmartDashboard.putData("balance code", m_balance.withTimeout(9));
    SmartDashboard.putData("resetEncArm",m_armAndJoint.resetArm());
    // m_driverController.a().toggleOnTrue(m_armAndJoint.PIDArmAndJoint(4,0));
    // m_driverController.a().toggleOnTrue(m_armAndJoint.PIDArmAndJoint(2,-10));a

 }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    //return null;
   // return m_chooser.getSelected();
    // An example command will be run in autonomous
    //return m_armAndJoint.PIDArmAndJoint(3, 2);
   //return a;
   return followTrajectoryCommand(examplePath, true);
    //return m_chooser.getSelected();
    //return m_autoCommand;
    // for unit testing 
    //return new Balancing(m_drivebase)
    //return m_turnToTarget;

  }
}
