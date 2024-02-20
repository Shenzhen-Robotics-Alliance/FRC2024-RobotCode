package frc.robot.Utils.Tests;

import edu.wpi.first.wpilibj.PWM;

public class LEDTest implements SimpleRobotTest {// a function that can make the light change 3 colors, the light also need to be able to blink
    PWM testpwm = new PWM(1);//0:, 1:blue, 2:green

    @Override
    public void testStart() {
        // TODO Auto-generated method stub
        System.out.println("<-- LED Test | Testing... -->");
        //testpwm.setAlwaysHighMode();
        testpwm.close();;
    
    }

    @Override
    public void testPeriodic() {
        // TODO Auto-generated method stub
        
    }
    
}
