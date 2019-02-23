/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import javax.management.RuntimeErrorException;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
/**
 * Add your docs here.
 */
public class ElevatorMechanism extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  public TalonSRX elevatorMotor1 = new TalonSRX(RobotMap.p_elevatormotor1);
  public TalonSRX elevatorMotor2 = new TalonSRX(RobotMap.p_elevatormotor2);

  public DigitalInput upperLimit = new DigitalInput(RobotMap.p_frontUpperLimit);
  public DigitalInput lowerLimit = new DigitalInput(RobotMap.p_frontLowerLimit);

  public static double FLOOR_POSITION = 0;
  public static double CARGO1_POSITION = 1000;
  public static double PANEL1_POSITION = 2000;
  public static double CARGO2_POSITION = 3000;
  public static double PANEL2_POSITION = 4000;
  public static double CARGO3_POSITION = 5000;
  public static double PANEL3_POSITION = 6000;
  public static double targetPosition = 1;
  boolean atBottom = false;
  static final double KP = 0.4;
  static final double KI = 0.0;
  static final double KD = 0.00;
  static final double KF = 0.02;
  static final double KToleranceDegrees = 2.0f;
  static final int SLOT_IDX = 0;
  static final int kTimeoutMS = 30;
  // public Encoder liftEncoder = new Encoder(RobotMap.p_encoderchannel1, RobotMap.p_encoderchannel2, false, Encoder.EncodingType.k4X);
  // public FeedbackDevice encoder = FeedbackDevice.QuadEncoder;

  public ElevatorMechanism(){
    // liftEncoder.setDistancePerPulse(1000);
    // liftEncoder.reset();
    // elevatorMotor1.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    // elevatorMotor2.follow(elevatorMotor1);
    // elevatorMotor2.setInverted(true);
    // // elevatorMotor1.setInverted(true);
    // elevatorMotor1.setNeutralMode(NeutralMode.Brake);
    // elevatorMotor2.setNeutralMode(NeutralMode.Brake);
    elevatorMotor1.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, kTimeoutMS);
    elevatorMotor2.follow(elevatorMotor1);
    elevatorMotor2.setInverted(true);
    elevatorMotor1.setNeutralMode(NeutralMode.Brake);
    elevatorMotor2.setNeutralMode(NeutralMode.Brake);
    elevatorMotor1.configNominalOutputForward(0, kTimeoutMS);
    elevatorMotor1.configNominalOutputReverse(0, kTimeoutMS);
    elevatorMotor1.configPeakOutputForward(0.5, kTimeoutMS);
    elevatorMotor1.configPeakOutputReverse(-0.5, kTimeoutMS);
    elevatorMotor1.selectProfileSlot(SLOT_IDX, 0);
    elevatorMotor1.config_kF(SLOT_IDX, KF, kTimeoutMS);
    elevatorMotor1.config_kP(SLOT_IDX, KP, kTimeoutMS);
    elevatorMotor1.config_kI(SLOT_IDX, KI, kTimeoutMS);
    elevatorMotor1.config_kD(SLOT_IDX, KD, kTimeoutMS);
    elevatorMotor1.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, kTimeoutMS);
    elevatorMotor1.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, kTimeoutMS);
  }
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
  public void resetEncoder(){
    // liftEncoder.reset();
    elevatorMotor1.setSelectedSensorPosition(0, 0, kTimeoutMS);
  }
  public double getEncoderCount(){
    // return liftEncoder.get();
    return elevatorMotor1.getSelectedSensorPosition();
  }
  public void setEncoderDestination(double counts){
    // elevatorMotor1.setSelectedSensorPosition(counts);
    targetPosition = counts;
    SmartDashboard.putNumber("Encoder Destination:  ", counts);
    maintainPosition();
  }

  public void maintainPosition(){
    // elevatorMotor1.setSelectedSensorPosition(counts);
    if(targetPosition>getEncoderCount()+100){
      // setSpeed(0.2);
      setPosition(targetPosition);
    }
    else if(targetPosition>=getEncoderCount()-100 && targetPosition<=getEncoderCount()+100){
      // setSpeed(0);
      setPosition(targetPosition);
      if(targetPosition == FLOOR_POSITION){
        resetEncoder();
      }
    }
    else {
      // setSpeed(-0.2);
      setPosition(targetPosition);
    }
    SmartDashboard.putNumber("Elevator Position:  ", getEncoderCount());
  }

  public void setSpeed(double speed){
    // if(!upperLimit.get()&&!lowerLimit.get()){
      // elevatorMotor1.set(speed);
    // }
    // else elevatorMotor1.set(0.0); 
    
  }
  public void setPosition(double counts){
    elevatorMotor1.set(ControlMode.Position, counts);
  }

  // public void goFloor(){
  //   setEncoderDestination(FLOOR_POSITION);
  // }

  public boolean setZeroPosition(){
    if(!atBottom){
      if(upperLimit.get()){
        // elevatorMotor1.set(0.0);
        // elevatorMotor2.set(0.0);
        throw new RuntimeErrorException( new Error("Elevator at top during initialization "));
      }
      else if(lowerLimit.get()){
        // elevatorMotor1.set(0.0);
        // elevatorMotor2.set(0.0);
        atBottom = true;
      }
      else {
        setSpeed(-0.15);
      }
    }
    if(atBottom){
      setEncoderDestination(500);
      if(getEncoderCount()<=600 && getEncoderCount()>= 400){
        setSpeed(0.0);
        resetEncoder();
        atBottom = false;
        return  true;
      }
    }
    
    return false;
    
  }
  // public double setUpperLimit(){
  //   if(){

  //   }
  // }

}
