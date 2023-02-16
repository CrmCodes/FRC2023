// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PIDArmSubsystem extends SubsystemBase {
  // private TalonSRX m_armMotor1 = new TalonSRX(15);
  // private TalonSRX m_armMotor2 = new TalonSRX(16);
  // private Encoder m_armEncoder = new Encoder(1, 2);
  private CANSparkMax m_armMotor =new CANSparkMax(18, MotorType.kBrushless);

  private RelativeEncoder m_SparkEncoder;

  private PIDController m_armPID = new PIDController(0.5, 0, 0); //2.5, 0.2, 0.5
  private SlewRateLimiter m_armRateLimiter = new SlewRateLimiter(1);
  private double kArmSetpoint = 0;
  private double convertedArmPos;
  private double output;
  private double kP, kI, kD;

  private final DigitalInput m_MagneticSwitch = new DigitalInput(4);
  private  boolean hasReset = false;
  /** Creates a new LinkageSubsystem. */
  public PIDArmSubsystem() {
    // m_armMotor2.setInverted(true);
    // m_armMotor1.setInverted(false);
    // // m_armEncoder.setDistancePerPulse(360./1316.);
    // m_armEncoder.reset();
    hasReset = false;
    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // convertedArmPos = m_armEncoder.getDistance() * 360 / 1316;

    m_SparkEncoder.setPositionConversionFactor(360/100);

    SmartDashboard.putNumber("armEncoder", convertedArmPos);
    SmartDashboard.putNumber("armSetpoint", kArmSetpoint);
    SmartDashboard.putBoolean("armMag", m_MagneticSwitch.get());

    double p = SmartDashboard.getNumber("P Gain", 0);
    double i = SmartDashboard.getNumber("I Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if((p != kP)) { m_armPID.setP(p); kP = p; }
    if((i != kI)) { m_armPID.setI(i); kI = i; }
    if((d != kD)) { m_armPID.setD(d); kD = d; }

    if(m_MagneticSwitch.get() == false && hasReset == false){
      // m_armEncoder.reset();
      m_SparkEncoder.setPosition(0);
      hasReset = true;
    }

    if (convertedArmPos > -20 && convertedArmPos < 20){
      output = MathUtil.clamp((m_armPID.calculate(Math.toRadians(convertedArmPos), Math.toRadians(kArmSetpoint))), -1.0, 1.0);
    } else if (convertedArmPos > 0){
      output = MathUtil.clamp((m_armPID.calculate(Math.toRadians(convertedArmPos), Math.toRadians(kArmSetpoint))), -0.5, 1.0);
    } else if (convertedArmPos < 0){
      output = MathUtil.clamp((m_armPID.calculate(Math.toRadians(convertedArmPos), Math.toRadians(kArmSetpoint))), -1.0, 0.5);
    }

    output = -1 * m_armRateLimiter.calculate(output);
      // m_armMotor1.set(ControlMode.PercentOutput, output);
      // m_armMotor2.set(ControlMode.PercentOutput, output);
      m_armMotor.set(output);
  }

  public void setPos(double setpoint){
    this.kArmSetpoint = setpoint;
  }

  public double getSetPoint(){
    return this.kArmSetpoint;
  }

  public double getPosition(){
    return convertedArmPos;
  }

  public void resetEncoder(){
    // m_armEncoder.reset();
    m_SparkEncoder.setPosition(0);
  }

  public void resetInitial(){
    hasReset = false;
  }
}
