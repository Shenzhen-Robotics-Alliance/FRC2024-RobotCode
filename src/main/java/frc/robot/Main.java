package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * The entry of the robot's program
 * 
 * @author Sam
 * @version 0.1
 */
public class Main {
    public static void main(String[] args) {
        System.out.printf("@@@@@@  void main(String[] args)  %n");
        RobotBase.startRobot(RobotShell::new);   // https://www.geeksforgeeks.org/double-colon-operator-in-java/
        // 可以ctrl点进上面RobotSheLL里面看源码。
        // 如果是decompiled的源码，intellij
        // idea有下载源文件选项（不知道vscode有没有）。
        // 下了源文件可以看库里面的javadoc
    }
}