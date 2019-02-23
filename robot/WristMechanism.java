/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANEncoder;

import edu.wpi.first.wpilibj.Encoder;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Add your docs here.
 */
public class WristMechanism extends Subsystem implements PIDOutput{
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  // public Encoder wristEncoder = new Encoder(RobotMap.p_secondencoderchannel1, RobotMap.p_secondencoderchannel2, false, Encoder.EncodingType.k4X);
  public TalonSRX wristMotor1 = new TalonSRX(RobotMap.p_wristmotor1);
  public TalonSRX wristMotor2 = new TalonSRX(RobotMap.p_wristmotor2);

  public WPI_TalonSRX intakeMotor = new WPI_TalonSRX(RobotMap.p_intakemotor);

  public double speed = 0.5;

  public PIDController wristController;

  static final double KP = 0.2;
  static final double KI = 0.00;
  static final double KD = 0.00;
  static final double KF = 0.2;
  static final double KToleranceDegrees = 2.0f;
  static final int SLOT_IDX = 0;

  public double turn45Encoder = 500;
  public double turn90Encoder = 0;
  public double turn180Encoder = -1000;

  public WristMechanism(){
    wristMotor1.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    wristMotor2.follow(wristMotor1);
    wristMotor2.setInverted(true);
    wristMotor1.setNeutralMode(NeutralMode.Brake);
    wristMotor2.setNeutralMode(NeutralMode.Brake);
    wristMotor1.configNominalOutputForward(0);
    wristMotor1.configNominalOutputReverse(0);
    wristMotor1.configPeakOutputForward(1);
    wristMotor1.configPeakOutputReverse(-1);
    wristMotor1.selectProfileSlot(SLOT_IDX, 0);
    wristMotor1.config_kF(SLOT_IDX, KF);
    wristMotor1.config_kP(SLOT_IDX, KP);
    wristMotor1.config_kI(SLOT_IDX, KI);
    wristMotor1.config_kD(SLOT_IDX, KD);
    

  }
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
  public void init(PIDOutput wrist){
    // wristController = new PIDController(kP, kI, kD, getEncoderCount(), wrist);
    //   wristController.setInputRange(-180.0f,  180.0f);
    //   wristController.setOutputRange(-0.15, 0.15);
    //   wristController.setAbsoluteTolerance(kToleranceDegrees);
    //   wristController.setContinuous(true);
  }
  public void resetEncoder(){
    wristMotor1.setSelectedSensorPosition(0);
  }
  public void moveWristMotor(double speed){
    wristMotor1.set(ControlMode.Position, speed);
  }
  public void moveWristMotorCounts(double counts){
    // wristMotor1.set(ControlMode.Position, counts);
    wristMotor1.set(ControlMode.MotionMagic, counts);
  }
  public void intakeIn(){
    intakeMotor.set(speed);
  }
  public void intakeOut(){
    intakeMotor.set(-speed);
  }
  public void stopIntake(){
    intakeMotor.set(0.0);
  }

  public double getEncoderCount(){
    return wristMotor1.getSelectedSensorPosition();
  }

  public boolean moveWrist45(){
    if(getEncoderCount()>turn45Encoder+100){
      // moveWristMotor(-0.15);
      moveWristMotorCounts(turn45Encoder);
      return false;
    }
    else if(getEncoderCount()<turn45Encoder-100){
      // moveWristMotor(0.15);
      moveWristMotorCounts(turn45Encoder);
      return false;
    }
    else {
      // moveWristMotor(0.0);
      return true;
    }
  }
  public boolean moveWrist90(){
    if(getEncoderCount()>turn90Encoder+100){
      // moveWristMotor(-0.15);
      moveWristMotorCounts(turn90Encoder);
      return false;
    }
    else if(getEncoderCount()<turn90Encoder-100){
      // moveWristMotor(0.15);
      moveWristMotorCounts(turn90Encoder);
      return false;
    }
    else {
      // moveWristMotor(0.0);
      return true;
    }
  }
  public boolean moveWrist180(){
    if(getEncoderCount()>turn180Encoder+100){
      // moveWristMotor(-0.15);
      moveWristMotorCounts(turn180Encoder);
      return false;
    }
    else if(getEncoderCount()<turn180Encoder-100){
      // moveWristMotor(0.15);
      moveWristMotorCounts(turn180Encoder);
      return false;
    }
    else {
      // moveWristMotor(0.0);
      return true;
    }
  }
    @Override
  public void pidWrite(double output) {
    speed = output;
    SmartDashboard.putNumber("WristMotor PidWrite:  ", output);
  }
  public void turnAngle(){
     moveWristMotor(speed);
  }
}
