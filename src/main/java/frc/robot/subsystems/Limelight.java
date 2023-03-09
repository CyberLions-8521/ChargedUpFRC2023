// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public Limelight() {
    CameraServer.startAutomaticCapture();
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public CommandBase exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = table.getEntry("ta");
  NetworkTableEntry tv = table.getEntry("tv");
  NetworkTableEntry pose = table.getEntry("botpose");
  NetworkTableEntry pose_blue = table.getEntry("botpose_wpiblue");
  NetworkTableEntry pose_red = table.getEntry("botpose_wpired");
  NetworkTableEntry current_pipe = table.getEntry("getpipe");


  public double getTx(){
    return tx.getDouble(0.0);
  }

  public double getTy(){
    return ty.getDouble(0.0);
  }

  public double getTa(){
    return ta.getDouble(0.0);
  }

  public double getTv(){
    return tv.getDouble(0.0);
  }

  public void setPipeline(int target){
    table.getEntry("pipeline").setNumber(target);
  }

  public boolean getDetectedTarget() {
    return (tv.getDouble(0.0) == 1);
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    getDetectedTarget();
    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double v = tv.getDouble(0.0);
    double area = ta.getDouble(0.0);
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);
    SmartDashboard.putNumber("pipe", NetworkTableInstance.getDefault().getTable("limelight").getEntry("getpipe").getDouble(0)); 
    SmartDashboard.putBoolean("Target detected", getDetectedTarget());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
