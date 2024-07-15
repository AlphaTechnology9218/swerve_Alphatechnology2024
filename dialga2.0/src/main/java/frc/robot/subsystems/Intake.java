package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase{

        private final int IntakeMotorID = 8;
        

        CANSparkMax IntakeMotor = new CANSparkMax(IntakeMotorID, MotorType.kBrushed);
        

        
        public void IntakeDrive(double val){
            IntakeMotor.set(val);
        }

      
        public void IntakeStop(){
            IntakeMotor.set(0);
        }
}