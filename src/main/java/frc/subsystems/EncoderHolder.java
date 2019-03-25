/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.subsystems;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.Encoder;


public class EncoderHolder extends Subsystem{
    public static final int elevatorSparkPort = 2;
    private Spark elevatorSpark;
    private PIDController pid;
    private Encoder elevatorEncoder = new Encoder(1, 2, false, EncodingType.k4X);

    //PID Constants
    double kP = 1.2;
    double kI = 0.7;
    double kD = 0.3;

    // Constants
    public static final double kDistancePerRevolution = 18.84;  // guestimate from your code
    public static final double kPulsesPerRevolution = 1024;     // for an AS5145B Magnetic Encoder
    public static final double kDistancePerPulse = kDistancePerRevolution / kPulsesPerRevolution;

public EncoderHolder(){
    //Instantiate SPX
    elevatorSpark = new Spark(elevatorSparkPort);
    //Instantiate PID
//    pid = new PIDController(kP, kI, kD, ElevatorSPX, ElevatorSPX);
}

public void runPID(double targetPos){
    pid.disable();
    pid.setSetpoint(targetPos);
    pid.enable();
}

public void setPower(double pow){
    pid.disable();
    elevatorSpark.set(pow);
}

public double getEncoderVal(){
    return elevatorSpark.getPosition();
}

protected void initDefaultCommand() {}

}