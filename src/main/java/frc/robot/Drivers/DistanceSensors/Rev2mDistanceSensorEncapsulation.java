 package frc.robot.Drivers.DistanceSensors;

 import com.revrobotics.Rev2mDistanceSensor;

 public class Rev2mDistanceSensorEncapsulation implements DistanceSensor {
     private static final Rev2mDistanceSensor rev2mDistanceSensorInstance = new Rev2mDistanceSensor(Rev2mDistanceSensor.Port.kOnboard);
     public Rev2mDistanceSensorEncapsulation() {
         rev2mDistanceSensorInstance.setMeasurementPeriod(0.02);
         setEnabled(true);
     }
     @Override
     public double getDistanceCM() {
         // System.out.println("distance sensor update rate: " + 1/rev2mDistanceSensorInstance.getMeasurementPeriod());
         if (!rev2mDistanceSensorInstance.isRangeValid()) {
             System.out.println("range invalid, raw reading: " + rev2mDistanceSensorInstance.getRange(Rev2mDistanceSensor.Unit.kMillimeters));
             return Double.POSITIVE_INFINITY;
         }
         return rev2mDistanceSensorInstance.getRange(Rev2mDistanceSensor.Unit.kMillimeters) / 10.0;
     }

     @Override
     public void setEnabled(boolean enabled) {
         rev2mDistanceSensorInstance.setEnabled(enabled);
         rev2mDistanceSensorInstance.setAutomaticMode(enabled);
     }
 }
