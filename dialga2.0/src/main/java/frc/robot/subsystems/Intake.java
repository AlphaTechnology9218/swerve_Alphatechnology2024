package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Intake extends SubsystemBase{

        private final int IntakeMotorID = 18;
        private final int SensorAnalogChannel = 3;
        
        AnalogPotentiometer intakeInfraRedSensor = new AnalogPotentiometer(SensorAnalogChannel, 5, -0.4);

        CANSparkMax IntakeMotor = new CANSparkMax(IntakeMotorID, MotorType.kBrushed);
        

        
        public void IntakeDrive(double val){
            IntakeMotor.set(val);
            
        }

      
        public void IntakeStop(){
            IntakeMotor.set(0);
            
        }

        public double getCurrent(){
            return IntakeMotor.getOutputCurrent();
        }

        public boolean gamePieceIn(){
            double sensorVal = intakeInfraRedSensor.get();
            boolean in = false;
            if(sensorVal < 1 ){
                in = false;
            }else if(sensorVal > 1){
                in = true;
            }
            return in;
        }
        
        public double getIntakeSensorVal(){
            double val = intakeInfraRedSensor.get();
            return val; 
        }

        @Override
        public void periodic() {
            SmartDashboard.putBoolean("Sensor Value Test", gamePieceIn());
            SmartDashboard.putNumber("Numerical Sensor Value Test", getIntakeSensorVal());
        }
}