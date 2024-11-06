package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;


public class Intake extends SubsystemBase{
        
        AnalogPotentiometer intakeInfraRedSensor = new AnalogPotentiometer(IntakeConstants.SensorAnalogChannel, 5, -0.4);
        CANSparkMax IntakeMotor = new CANSparkMax(IntakeConstants.IntakeMotorID, MotorType.kBrushed);

        public Intake(){
            IntakeStop();

            IntakeMotor.restoreFactoryDefaults();

            IntakeMotor.setIdleMode(IdleMode.kBrake);

            IntakeMotor.setSmartCurrentLimit(40, 60);

            IntakeMotor.setInverted(false);

            IntakeMotor.burnFlash();
        }
        
        public void IntakeDrive(double val){
            IntakeMotor.set(val);
            
        }

      
        public void IntakeStop(){
            IntakeMotor.set(0);
            
        }

        public double getCurrent(){
            return IntakeMotor.getOutputCurrent();
        }

        
        public double getIntakeSensorVal(){
            double val = intakeInfraRedSensor.get();
            return val; 
        }

        @Override
        public void periodic() {
            SmartDashboard.putNumber("Numerical Sensor Value Test", getIntakeSensorVal());
        }
}