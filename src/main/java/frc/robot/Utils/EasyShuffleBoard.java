package frc.robot.Utils;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

import java.util.HashMap;
import java.util.Map;

/**
 * an easier way to access custom tags of shuffleboard, but with the simple logic of SmartDashboard
 * by 5516 the "IRON MAPLE"
 * */
public class EasyShuffleBoard {
    private static final Map<String, Map<String, GenericEntry>> widgetsInTags = new HashMap<>();
    public static void putNumber(String tab, String title, double number) {
        try {
            if (!widgetsInTags.containsKey(tab))
                widgetsInTags.put(tab, new HashMap<>());
            if (!widgetsInTags.get(tab).containsKey(title))
                widgetsInTags.get(tab).put(title, Shuffleboard.getTab(tab).add(title, number).getEntry());
            widgetsInTags.get(tab).get(title).setDouble(number);
        } catch (Exception ignored) {
            // System.out.println("<-- shuffleboard error, ignoring... -->"); // TODO a lot of error here
        }

    }

    public static double getNumber(String tag, String title, double defaultValue) {
        if (!widgetsInTags.containsKey(tag) || !widgetsInTags.get(tag).containsKey(title))
            return defaultValue; // in case the widget is nowhere to be found
        try {
            return widgetsInTags.get(tag).get(title).getDouble(defaultValue);
        } catch (Exception e) {
            e.printStackTrace();
            return defaultValue;
        }

    }
}
