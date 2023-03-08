// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAlternateEncoder.Type;
import com.revrobotics.AbsoluteEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ArmAndJoint extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private final CANSparkMax m_jointMotor1 = new CANSparkMax(Constants.MotorControllerIDs.JOINT_MOTOR1, MotorType.kBrushless);
  private final CANSparkMax m_jointMotor2 = new CANSparkMax(Constants.MotorControllerIDs.JOINT_MOTOR2, MotorType.kBrushless);
  //private final SparkMaxLimitSwitch m_limitSwitch = m_jointMotor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
  public final CANSparkMax m_armMotor = new CANSparkMax(Constants.MotorControllerIDs.ARM_MOTOR, MotorType.kBrushless);
  public final AbsoluteEncoder m_jointEncoder = m_jointMotor2.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
  public final MotorControllerGroup m_jointGroup = new MotorControllerGroup(m_jointMotor1, m_jointMotor2);

  private final RelativeEncoder m_armEncoder = m_armMotor.getEncoder();
  //private final AnalogPotentiometer m_armPotentiometer = new AnalogPotentiometer(0, 933, -30);

  private final PIDController m_PIDArm = new PIDController(1, 0.08, 0);
  private final PIDController m_PIDJoint = new PIDController(0.015, 0.00004, 0.000001);
  double heightOfJoint = 1.1811;

  public ArmAndJoint() {
    //m_jointEncoder.setPosition(0);
    //m_jointMotor.setSoftLimit(SoftLimitDirection.kReverse, 0);
    m_armMotor.setSoftLimit(SoftLimitDirection.kReverse, 0);
    m_jointMotor1.setInverted(true);
    m_jointMotor1.setSoftLimit(SoftLimitDirection.kReverse, 0);
    m_jointMotor2.setSoftLimit(SoftLimitDirection.kReverse, 0);
    m_armEncoder.setPosition(0);
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

  public CommandBase retractArm() {
    return runOnce(
      () -> {
        m_armMotor.set(-0.25);
      });
  }

  public CommandBase extendArm() {
    return runOnce(
      () -> {
        m_armMotor.set(0.25);
      });
  }

  public CommandBase PIDArmAndJoint(double SPx, double SPy){
    return run(
      () -> {
        moveToAngleSetpoint(SPx, SPy);
        armExtrusionToSetpoint(SPx, SPy);
      }
    );
  }

  // public CommandBase movingEntireArm(double angle, double meters){
  //   return run(
  //     () -> {
  //       moveToAngleSetpointRACHEL(angle);
  //       armSetpoint(meters);
  //     }
  //   )
  // }

  public CommandBase moveToAngle(double setpoint){
    return run(
      () -> {
        moveToAngleSetpointRACHEL(setpoint);
      }
    );
  }

  public CommandBase moveArm(double meters) {
    return run (
      ()-> {
        armSetpoint(meters);
      }
    );
  }

  public CommandBase resetArm() {
    return run (
      ()-> {
        m_armEncoder.setPosition(0);
      }
    );
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
/* 
  public boolean isLimitSwitchPressed(){
    return m_limitSwitch.isPressed();
  }*/

  public void move(double leftTrigger, double rightTrigger, boolean rightBumper, boolean leftBumper ){
    //softLimit(0.05, leftTrigger, rightTrigger);

    softLimit(0.343, 0.74, m_jointEncoder.getPosition());

    m_jointGroup.set((-leftTrigger + rightTrigger) * 0.1 + 0.02);
    if(rightBumper) {
      m_armMotor.set(0.5);
    } else if(leftBumper) {
      m_armMotor.set(-0.5);
    } else {
      m_armMotor.set(0);
    }
  }

  public double getCurrentAngle(){
    double revolutions = m_jointEncoder.getPosition();
    double angle = 0;
    final double m = -215.206;
    final double b = 180;
    angle = m * revolutions + b;
    return angle;
  }

  public void softLimit(double upper, double lower, double input){
    while(upper > m_jointEncoder.getPosition()){
      m_jointGroup.set(0);
    }
    while(lower < m_jointEncoder.getPosition()){
      m_jointGroup.set(0.1);
      if(lower-0.03>= m_jointEncoder.getPosition()){
        break;
      }
    }
  }

  public double getR2Length(){
    final double m = 0.00673312;
    final double b = 0.0287449;
    return m * m_armEncoder.getPosition() + b;
  }

  public double getR3Length(){
    return (getR2Length() + Constants.ElevatorConstants.r1);
  }

  public double distanceFormula(double x1, double x2, double y1, double y2){
    return ( Math.sqrt(Math.pow((x2 - x1),2) + Math.pow((y2 - y1),2)));
  }

  public void moveToAngleSetpoint(double SPx, double SPy){
    double output = m_PIDJoint.calculate(getCurrentAngle(), getAngleToSetpoint(SPx, SPy));
    m_jointGroup.set(output);
    SmartDashboard.putNumber("Angle Distance to Setpoint", getAngleToSetpoint(SPx, SPy));
  }

  public void moveToAngleSetpointBigK(double SPx, double SPy){
    double setpoint = getAngleToSetpoint(SPx, SPy);
    double output = m_PIDJoint.calculate(getCurrentAngle(), setpoint);
    m_jointGroup.set(output);
  }

  public void moveToAngleSetpointRACHEL(double wantedAngle){
    double output = m_PIDJoint.calculate(getCurrentAngle(), wantedAngle);
    m_jointGroup.set(output);
    softLimit(0.0125, 0.4, m_jointEncoder.getPosition());
    SmartDashboard.putNumber("output of angle motor", output);
  }

  public void armExtrusionToSetpoint(double SPx, double SPy){
    double distanceToSetpoint = distanceFormula(SPx, 0, heightOfJoint, SPy);
    double output = m_PIDArm.calculate(getR3Length(), distanceToSetpoint);
    m_armMotor.set(output);
    //SmartDashboard.putNumber("Arm length to Setpoint",  distanceToSetpoint);
  }

  public void armSetpoint(double m){
    double output = m_PIDArm.calculate(getR2Length(),m);
    m_armMotor.set(output);
  }

  public double getAngleToSetpoint(double SPx, double SPy){
    double angleToSetpoint = Math.toDegrees(Math.atan((SPy-heightOfJoint)/SPx))+90;    SmartDashboard.putNumber("Angle to Setpoint", angleToSetpoint);
    return angleToSetpoint;
  }

  public void PolarToXY(){
    double x = getR3Length() * (Math.cos(Math.toRadians(getCurrentAngle()-90)));
    double y = (getR3Length() * (Math.sin(Math.toRadians(getCurrentAngle()-90))) + heightOfJoint);
    double dist = distanceFormula(x, 0, y, heightOfJoint);
    SmartDashboard.putNumber("XArm", x);
    SmartDashboard.putNumber("YArm", y);  
    SmartDashboard.putNumber("distance", dist);

  }
  //(0.69,0.9) h1
  //(1.05,1.17) h2
  @Override
  public void periodic() {
    PolarToXY();
    //SmartDashboard.putNumber("Raw Joint Value", m_jointEncoder.getPosition());
    SmartDashboard.putNumber("Angle of Joint", getCurrentAngle());
    SmartDashboard.putNumber("Absolute Encoder Values", m_jointEncoder.getPosition());
    SmartDashboard.putNumber("R2 Length", getR2Length());
    SmartDashboard.putNumber("R3Length", getR3Length());
    SmartDashboard.putNumber("Arm Encoder Readings", m_armEncoder.getPosition());
    SmartDashboard.putNumber("lkafja;dskfk", m_PIDJoint.calculate(getCurrentAngle(), 45));
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
