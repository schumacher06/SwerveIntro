package frc.robot.subsystems;

import javax.swing.table.TableColumn;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.RobotDriveBase.MotorType;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ModuleConstants;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.utils.Controller;
import frc.robot.utils.Limelight;

public class SwerveModule{

    private final WPI_TalonFX driveMotor;
    private final Talon turningMotor;

    private final Talon driveEncoder;
    private final Talon turningEncoder;

    private final PIDController turningPidController;

    private final AnalogInput absoluteEncoder;
    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;

    public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed, 
        int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed){
          
        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;
        absoluteEncoder = new AnalogInput(absoluteEncoderId);

        driveMotor = new Talon(driveMotorId, MotorType.kBrushless);
        turningMotor = new Talon(turningMotorId, MotorType.kBrushless);


        driveMotor.setInverted(driveMotorReversed);
        turningMotor.setInverted(turningMotorReversed);

        driveEncoder = driveMotor.getEncoder();
        turningEncoder = turningMotor.getEncoder();

        driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
        driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
        turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);
        turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);

        turningPidController = new PIDController(ModuleConstants.kPTurning, 0, 0);
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);

        resetEncoders();
        }
        
        public double getDrivePosition(){
            return driveEncoder.getPosition();
        }
        public double getTurningPosition(){
            return turningEncoder.getPosition();
        }
        public double getDriveVelocity(){
            return driveEncoder.getVelocity();
        }
        public double getTurningVelocity(){
            return turningEncoder.getVelocity();
        }
        public double getAbsoluteEncoderRad(){
            double angle = absoluteEncoder.getVoltage() / RobotController.getVoltage5V();
            angle *= 2.0 * Math.PI;
            angle -= absoluteEncoderOffsetRad;
            return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
        }
        public void resetEncoders(){
            driveEncoder.setPosition(0);
            turningEncoder.setPosition(getAbsoluteEncoderRad());
        }
        public SwerveModuleState getState(){
            return new SwerveModuleState(getDriveVelocity(),new Rotation2d(getTurningPosition()));
        }
       public void setDesiredState(SwerveModuleState state){
           if(Math.abs(state.speedMetersPerSecond) < 0.001){
                stop();
                return;
           }
           state = SwerveModuleState.optimize(state, getState().angle);
           driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersperSecond);
           turningMotor.set(turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));
           SmartDashboard.putString("Swerve[" + absoluteEncoder.getChannel() + "] state", state.toString());

       } 
       public void stop(){
           driveMotor.set(0);
           turningMotor.set(0);
       }
    
}