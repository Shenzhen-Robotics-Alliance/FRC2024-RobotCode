 package frc.robot.Drivers.DistanceSensors;

 import com.revrobotics.Rev2mDistanceSensor;

 public class Rev2mDistanceSensorEncapsulation implements DistanceSensor {
     private final Rev2mDistanceSensor rev2mDistanceSensorInstance;
     public Rev2mDistanceSensorEncapsulation() {
         this.rev2mDistanceSensorInstance = new Rev2mDistanceSensor(Rev2mDistanceSensor.Port.kOnboard);
         setEnabled(true);
     }
     @Override
     public double getDistanceCM() {
         if (!rev2mDistanceSensorInstance.isRangeValid())
             return Double.POSITIVE_INFINITY;
         return rev2mDistanceSensorInstance.getRange(Rev2mDistanceSensor.Unit.kMillimeters) / 10.0;
     }

     @Override
     public void setEnabled(boolean enabled) {
         rev2mDistanceSensorInstance.setAutomaticMode(enabled);
     }
 }
