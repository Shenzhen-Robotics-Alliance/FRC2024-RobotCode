package frc.robot.Utils;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Modules.PositionReader.PositionEstimator;
import frc.robot.Modules.Chassis.SwerveBasedChassis;
import frc.robot.RobotCore;
import frc.robot.Utils.MathUtils.BezierCurve;
import frc.robot.Utils.MathUtils.Rotation2D;
import frc.robot.Utils.MathUtils.Vector2D;
import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.json.simple.parser.ParseException;

import java.io.*;
import java.util.ArrayList;
import java.util.List;

// TODO speed curves
public class SequentialCommandFactory {
    private final SwerveBasedChassis chassis;
    private final PositionEstimator positionEstimator;
    private final Vector2D robotStartingPosition;
    private final Rotation2D robotStartingRotation2D;

    public SequentialCommandFactory(RobotCore robotCore) {
        this(robotCore, new Vector2D(), new Rotation2D(0));
    }

    public SequentialCommandFactory(RobotCore robotCore, Vector2D robotStartingPosition, Rotation2D robotStartingRotation2D) {
        this.chassis = robotCore.chassisModule;
        this.positionEstimator = robotCore.positionReader;
        this.maintainCurrentRotation = () -> new Rotation2D(positionEstimator.getRobotRotation());
        this.robotStartingPosition = robotStartingPosition;
        this.robotStartingRotation2D = robotStartingRotation2D;
    }

    private static final SequentialCommandSegment.InitiateCondition justGo = () -> true;
    private static final Runnable doNothing = () -> {};
    private static final SequentialCommandSegment.IsCompleteChecker weDoNotCareAboutIsItComplete = () -> true;
    private final SequentialCommandSegment.RotationFeeder maintainCurrentRotation;
    private static final SequentialCommandSegment.RotationFeeder weDoNotCareAboutRotation = () -> null;

    public SequentialCommandSegment calibratePositionEstimator() {
        return justDoIt(() -> {
            positionEstimator.setRobotRotation(robotStartingRotation2D.getRadian());
            positionEstimator.setRobotPosition(robotStartingPosition);
        });
    }

    public SequentialCommandSegment moveToPoint(Vector2D destination) {
        return moveToPoint(destination, doNothing, doNothing, doNothing);
    }

    public SequentialCommandSegment moveToPoint(Vector2D destination, Runnable beginning, Runnable periodic, Runnable ending) {
        return moveToPointIf(justGo, destination, beginning, periodic, ending);
    }

    public SequentialCommandSegment moveToPointIf(SequentialCommandSegment.InitiateCondition initiateCondition, Vector2D destination, Runnable beginning, Runnable periodic, Runnable ending) {
        return new SequentialCommandSegment(
                initiateCondition,
                () -> new BezierCurve(positionEstimator.getRobotPosition2D(), destination),
                beginning, periodic, ending,
                chassis::isCurrentTranslationalTaskFinished,
                maintainCurrentRotation, maintainCurrentRotation
        );
    }

    public SequentialCommandSegment moveToPointIf(SequentialCommandSegment.InitiateCondition initiateCondition, Vector2D destination, Runnable beginning, Runnable periodic, Runnable ending, Rotation2D endingRotation) {
        return new SequentialCommandSegment(
                initiateCondition,
                () -> new BezierCurve(positionEstimator.getRobotPosition2D(), destination),
                beginning, periodic, ending,
                chassis::isCurrentTranslationalTaskFinished,
                maintainCurrentRotation, () -> endingRotation
        );
    }

    public SequentialCommandSegment moveToPointAndStop(Vector2D destination) {
        return moveToPointAndStopIf(justGo, destination);
    }
    public SequentialCommandSegment moveToPointAndStop(Vector2D destination, Runnable beginning, Runnable periodic, Runnable ending) {
        return moveToPointAndStopIf(justGo, destination, beginning, periodic, ending);
    }

    public SequentialCommandSegment moveToPointAndStopIf(SequentialCommandSegment.InitiateCondition initiateCondition, Vector2D destination) {
        return moveToPointAndStopIf(initiateCondition, destination, doNothing, doNothing, doNothing);
    }
    public SequentialCommandSegment moveToPointAndStopIf(SequentialCommandSegment.InitiateCondition initiateCondition, Vector2D destination, Runnable beginning, Runnable periodic, Runnable ending) {
        return new SequentialCommandSegment(
                initiateCondition,
                () -> new BezierCurve(positionEstimator.getRobotPosition2D(), destination),
                beginning, periodic, ending,
                chassis::isCurrentTranslationalTaskFinished,
                maintainCurrentRotation, maintainCurrentRotation
        );
    }

    public SequentialCommandSegment moveFromPointToPoint(Vector2D startingPoint, Vector2D endingPoint) {
        return moveFromPointToPointIf(justGo, startingPoint, endingPoint, doNothing, doNothing, doNothing, maintainCurrentRotation, maintainCurrentRotation);
    }

    public SequentialCommandSegment moveFromPointToPoint(Vector2D startingPoint, Vector2D endingPoint, Rotation2D startingRotation, Rotation2D endingRotation) {
        return moveFromPointToPoint(startingPoint, endingPoint, doNothing, doNothing, doNothing, startingRotation, endingRotation);
    }

    public SequentialCommandSegment moveFromPointToPoint(Vector2D startingPoint, Vector2D endingPoint, Runnable beginning, Runnable periodic, Runnable ending, Rotation2D startingRotation, Rotation2D endingRotation) {
        return moveFromPointToPointIf(justGo, startingPoint, endingPoint, beginning, periodic, ending, () -> startingRotation, () -> endingRotation);
    }

    public SequentialCommandSegment moveFromPointToPointIf(SequentialCommandSegment.InitiateCondition initiateCondition, Vector2D startingPoint, Vector2D endingPoint, Runnable beginning, Runnable periodic, Runnable ending, SequentialCommandSegment.RotationFeeder startingRotationFeeder, SequentialCommandSegment.RotationFeeder endingRotationFeeder) {
        return new SequentialCommandSegment(
                initiateCondition,
                () -> new BezierCurve(startingPoint, endingPoint),
                beginning, periodic, ending,
                weDoNotCareAboutIsItComplete,
                startingRotationFeeder, endingRotationFeeder
        );
    }

    public SequentialCommandSegment moveFromPointToMidPointToPoint(Vector2D startingPoint, Vector2D midPoint, Vector2D endingPoint) {
        return moveFromPointToMidPointToPointIf(justGo, startingPoint, midPoint, endingPoint, doNothing, doNothing, doNothing, maintainCurrentRotation, maintainCurrentRotation);
    }

    public SequentialCommandSegment moveFromPointToMidPointToPoint(Vector2D startingPoint, Vector2D midPoint, Vector2D endingPoint, Rotation2D startingRotation, Rotation2D endingRotation) {
        return moveFromPointToMidPointToPoint(startingPoint, midPoint, endingPoint, doNothing, doNothing, doNothing, startingRotation, endingRotation);
    }

    public SequentialCommandSegment moveFromPointToMidPointToPoint(Vector2D startingPoint, Vector2D midPoint, Vector2D endingPoint, Runnable beginning, Runnable periodic, Runnable ending, Rotation2D startingRotation, Rotation2D endingRotation) {
        return moveFromPointToMidPointToPointIf(justGo, startingPoint, midPoint, endingPoint, beginning, periodic, ending, () -> startingRotation, () -> endingRotation);
    }

    public SequentialCommandSegment moveFromPointToMidPointToPointIf(SequentialCommandSegment.InitiateCondition initiateCondition, Vector2D startingPoint, Vector2D midPoint, Vector2D endingPoint, Runnable beginning, Runnable periodic, Runnable ending, SequentialCommandSegment.RotationFeeder startingRotationFeeder, SequentialCommandSegment.RotationFeeder endingRotationFeeder) {
        return new SequentialCommandSegment(
                initiateCondition,
                () -> new BezierCurve(startingPoint, midPoint, endingPoint),
                beginning, periodic, ending,
                weDoNotCareAboutIsItComplete,
                startingRotationFeeder, endingRotationFeeder
        );
    }

    public SequentialCommandSegment moveFromPointToPointAndStop(Vector2D startingPoint, Vector2D endingPoint) {
        return moveFromPointToPointAndStopIf(justGo, startingPoint, endingPoint, doNothing, doNothing, doNothing, maintainCurrentRotation, maintainCurrentRotation);
    }

    public SequentialCommandSegment moveFromPointToPointAndStop(Vector2D startingPoint, Vector2D endingPoint, Rotation2D startingRotation, Rotation2D endingRotation) {
        return moveFromPointToPointAndStop(startingPoint, endingPoint, doNothing, doNothing, doNothing, startingRotation, endingRotation);
    }

    public SequentialCommandSegment moveFromPointToPointAndStop(Vector2D startingPoint, Vector2D endingPoint, Runnable beginning, Runnable periodic, Runnable ending, Rotation2D startingRotation, Rotation2D endingRotation) {
        return moveFromPointToPointAndStopIf(justGo, startingPoint, endingPoint, beginning, periodic, ending, () -> startingRotation, () -> endingRotation);
    }

    public SequentialCommandSegment moveFromPointToPointAndStopIf(SequentialCommandSegment.InitiateCondition initiateCondition, Vector2D startingPoint, Vector2D endingPoint, Runnable beginning, Runnable periodic, Runnable ending, SequentialCommandSegment.RotationFeeder startingRotationFeeder, SequentialCommandSegment.RotationFeeder endingRotationFeeder) {
        return new SequentialCommandSegment(
                initiateCondition,
                () -> new BezierCurve(startingPoint, endingPoint),
                beginning, periodic, ending,
                chassis::isCurrentTranslationalTaskFinished,
                startingRotationFeeder, endingRotationFeeder
        );
    }

    public SequentialCommandSegment moveFromPointToMidPointToPointAndStop(Vector2D startingPoint, Vector2D midPoint, Vector2D endingPoint) {
        return moveFromPointToMidPointToPointAndStopIf(justGo, startingPoint, midPoint, endingPoint, doNothing, doNothing, doNothing, maintainCurrentRotation, maintainCurrentRotation);
    }

    public SequentialCommandSegment moveFromPointToMidPointToPointAndStop(Vector2D startingPoint, Vector2D midPoint, Vector2D endingPoint, Rotation2D startingRotation, Rotation2D endingRotation) {
        return moveFromPointToMidPointToPointAndStop(startingPoint, midPoint, endingPoint, doNothing, doNothing, doNothing, startingRotation, endingRotation);
    }

    public SequentialCommandSegment moveFromPointToMidPointToPointAndStop(Vector2D startingPoint, Vector2D midPoint, Vector2D endingPoint, Runnable beginning, Runnable periodic, Runnable ending, Rotation2D startingRotation, Rotation2D endingRotation) {
        return moveFromPointToMidPointToPointAndStopIf(justGo, startingPoint, midPoint, endingPoint, beginning, periodic, ending, () -> startingRotation, () -> endingRotation);
    }

    public SequentialCommandSegment moveFromPointToMidPointToPointAndStopIf(SequentialCommandSegment.InitiateCondition initiateCondition, Vector2D startingPoint, Vector2D midPoint, Vector2D endingPoint, Runnable beginning, Runnable periodic, Runnable ending, SequentialCommandSegment.RotationFeeder startingRotationFeeder, SequentialCommandSegment.RotationFeeder endingRotationFeeder) {
        return new SequentialCommandSegment(
                initiateCondition,
                () -> new BezierCurve(startingPoint, midPoint, endingPoint),
                beginning, periodic, ending,
                chassis::isCurrentTranslationalTaskFinished,
                startingRotationFeeder, endingRotationFeeder
        );
    }


    public SequentialCommandSegment faceDirection(Rotation2D direction){
        return faceDirection(direction, doNothing, doNothing, doNothing);
    }

    public SequentialCommandSegment faceDirection(Rotation2D direction, Runnable beginning, Runnable periodic, Runnable ending) {

        return new SequentialCommandSegment(
                justGo,
                () -> null,
                beginning, periodic, ending,
                chassis::isCurrentRotationalTaskFinished,
                positionEstimator::getRobotRotation2D,
                () -> direction
        );
    }

    public SequentialCommandSegment lockChassis() {
        return lockChassisFor(99999);
    }
    public SequentialCommandSegment lockChassisFor(long timeMillis) {
        final Timer timer = new Timer(); timer.start();
        return new SequentialCommandSegment(
                justGo,
                () -> null,
                () -> {
                    chassis.setChassisLocked(true, null);
                    timer.reset();
                },
                doNothing,
                doNothing,
                () -> timer.hasElapsed(timeMillis / 1000.0),
                weDoNotCareAboutRotation, weDoNotCareAboutRotation
        );
    }
    public SequentialCommandSegment lockChassisIfAndUntil(SequentialCommandSegment.InitiateCondition initiateCondition, SequentialCommandSegment.IsCompleteChecker isCompleteChecker) {
        return new SequentialCommandSegment(
                initiateCondition,
                () -> null,
                () -> chassis.setChassisLocked(true, null),
                doNothing,
                doNothing,
                isCompleteChecker,
                weDoNotCareAboutRotation, weDoNotCareAboutRotation
        );
    }

    public SequentialCommandSegment justDoIt(Runnable job) {
        return new SequentialCommandSegment(
                justGo,
                () -> null,
                job,
                doNothing,
                doNothing,
                weDoNotCareAboutIsItComplete,
                weDoNotCareAboutRotation, weDoNotCareAboutRotation
        );
    }

    public List<BezierCurve> getBezierCurvesFromPathFile(String pathName) {
        try (BufferedReader br = new BufferedReader(new FileReader(new File(
                Filesystem.getDeployDirectory(), "pathplanner/paths/" + pathName + ".path")))) {
            StringBuilder fileContentBuilder = new StringBuilder();
            String line;
            while ((line = br.readLine()) != null) {
                fileContentBuilder.append(line);
            }
            String fileContent = fileContentBuilder.toString();
            JSONObject pathJson = (JSONObject) new JSONParser().parse(fileContent);
            JSONArray waypointsJson = (JSONArray) pathJson.get("waypoints");

            JSONArray rotationTargetsJson = (JSONArray) pathJson.get("rotationTargets");

            List<BezierCurve> curves = new ArrayList<>();
            for (int i = 0; i < waypointsJson.size() - 1; i++) {
                JSONObject point = (JSONObject) waypointsJson.get(i),
                        nextPoint = (JSONObject) waypointsJson.get(i+1);
                curves.add(new BezierCurve(
                        pointFromJson((JSONObject) point.get("anchor")),
                        pointFromJson((JSONObject) point.get("nextControl")),
                        pointFromJson((JSONObject) nextPoint.get("prevControl")),
                        pointFromJson((JSONObject) nextPoint.get("anchor"))
                ));
            }
            return curves;
        } catch (FileNotFoundException e) {
            throw new RuntimeException("Cannot Find Path File: " + pathName + " From Deploy Directory: " + Filesystem.getDeployDirectory());
        } catch (IOException e) {
            throw new RuntimeException("IO Error While Reading File: " + pathName);
        } catch (ParseException e) {
            throw new RuntimeException("Error Occurred While Processing JSON Path File: " + pathName);
        }
    }

    private static Vector2D pointFromJson(JSONObject pointJson) {
        final double x = ((Number) pointJson.get("x")).doubleValue();
        final double y = ((Number) pointJson.get("y")).doubleValue();

        DriverStation.Alliance alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Red);

        alliance = DriverStation.Alliance.Blue;

        final double fieldHeight = 8.21, fieldWidth = 16.54;
        return switch (alliance) {
            case Red -> new Vector2D(new double[]{y - fieldHeight / 2, fieldWidth - x});
            case Blue -> new Vector2D(new double[]{fieldHeight / 2 - y, x});
        };
    }
}
