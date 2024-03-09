package frc.robot.Utils.Tests;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.Utils.MathUtils.Vector2D;
import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.json.simple.parser.ParseException;

import java.io.*;

public class BezierTest implements SimpleRobotTest {
    @Override
    public void testStart() {
        System.out.println("<-- hello robot simulation! -->");
        final String pathName = "test path";
        try (BufferedReader br = new BufferedReader(new FileReader(new File(
                    Filesystem.getDeployDirectory(), "pathplanner/paths/" + pathName + ".path")))) {
            StringBuilder fileContentBuilder = new StringBuilder();
            String line;
            while ((line = br.readLine()) != null) {
                fileContentBuilder.append(line);
            }
            String fileContent = fileContentBuilder.toString();
            JSONObject pathJson = (JSONObject) new JSONParser().parse(fileContent);
            JSONArray waypointsJson = (JSONArray) pathJson.get("waypoints"),
                    rotationTargetsJson = (JSONArray) pathJson.get("rotationTargets");

            // First point
            JSONObject firstPoint = (JSONObject) waypointsJson.get(0);
            System.out.println("<-- first point anchor: " + pointFromJson((JSONObject) firstPoint.get("anchor")) + " -->");
            System.out.println("<-- first point nextControl: " + pointFromJson((JSONObject) firstPoint.get("nextControl")));
            for (int i = 1; i < waypointsJson.size() - 1; i++) {
                JSONObject point = (JSONObject) waypointsJson.get(i);
                System.out.println("<-- point " + i + " prevControl: " + pointFromJson((JSONObject) point.get("prevControl")));
                System.out.println("<-- point " + i + " anchor: " + pointFromJson((JSONObject) point.get("anchor")) + " -->");
                System.out.println("<-- point " + i + " nextControl: " + pointFromJson((JSONObject) point.get("nextControl")));
            }
        } catch (FileNotFoundException e) {
            throw new RuntimeException("Cannot Find Path File: " + pathName + " From Deploy Directory: " + Filesystem.getDeployDirectory());
        } catch (IOException e) {
            throw new RuntimeException("IO Error While Reading File: " + pathName);
        } catch (ParseException e) {
            throw new RuntimeException("Error Occurred While Processing JSON Path File: " + pathName);
        }
    }

    private static Vector2D pointFromJson(JSONObject pointJson) {
        double x = ((Number) pointJson.get("x")).doubleValue();
        double y = ((Number) pointJson.get("y")).doubleValue();

        DriverStation.Alliance alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Red);

        alliance = DriverStation.Alliance.Red;

        /* TODO process x and y */
        switch (alliance) {
            case Red -> {

            }
            case Blue -> {

            }
        }

        return new Vector2D(new double[] {x, y});
    }

    @Override
    public void testPeriodic() {

    }
}
