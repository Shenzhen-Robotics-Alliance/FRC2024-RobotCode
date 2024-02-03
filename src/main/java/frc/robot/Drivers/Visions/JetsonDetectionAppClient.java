package frc.robot.Drivers.Visions;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.net.*;
import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

public class JetsonDetectionAppClient implements RawObjectDetectionCamera {
    private final String name;
    private final String serverURL;
    private Thread communicationThread;
    private boolean activated;
    private String results;
    private List<ObjectTargetRaw> targetsList;

    private final double[] cameraResolution;


    public JetsonDetectionAppClient(String name, String jetsonIpAddress, int portID) {
        this(name, jetsonIpAddress, portID, new double[] {640, 480});
    }
    public JetsonDetectionAppClient(String name, String jetsonIpAddress, int portID, double[] cameraResolution) {
        this.name = name;
        this.serverURL = "http://" + jetsonIpAddress + ":" + portID;
        this.cameraResolution = cameraResolution;
        this.activated = false;
        this.results = "no-rst";
    }

    private void communicateAndUpdateContinuously() {
        while (activated) {
            // System.out.println("jetson client activated: " + activated);
            try {
                tryToEstablishConnectionWithServer();
            } catch (Exception e) {
                // System.out.println("<-- Jetson Client | Connection failed. Waiting for the server to start... -->");
                try {Thread.sleep(50);} catch (InterruptedException ignored) {}
            }
        }
    }

    private void tryToEstablishConnectionWithServer() throws InterruptedException, IOException {
        URL resultsURL = new URL(serverURL + "/results");
        HttpURLConnection connection = (HttpURLConnection) resultsURL.openConnection();
        connection.setConnectTimeout(500);
        connection.setRequestMethod("GET");
        try {
            pullResultsFromServer(connection);
        } catch (SocketTimeoutException e) {
            // System.out.println("<-- Jetson Client | Connection timeout. Waiting for the server to start... -->");
        } catch (ConnectException e) {
            // System.out.println("<-- Jetson Client | Connection error. Waiting for the server to start... -->");
        }

        connection.disconnect();
        Thread.sleep(500); // wait for the server to start
    }

    private void pullResultsFromServer(HttpURLConnection connectionWithServer) throws IOException {
        int responseCode = connectionWithServer.getResponseCode();

        if (responseCode == HttpURLConnection.HTTP_OK) {
            BufferedReader reader = new BufferedReader(new InputStreamReader(connectionWithServer.getInputStream()));
            String line;
            while (activated && (line = reader.readLine()) != null) {
                // System.out.println("<-- Jetson Client | Received Line: " + line + " -->");
                results = line;
            }
            reader.close();
        }
        else System.out.println("<-- Jetson Client | Server Response Error Code: " + responseCode + " -->");
    }

    @Override
    public void startRecognizing() {
        if (activated) return;
        this.activated = true;
        communicationThread = new Thread(this::communicateAndUpdateContinuously, name+"-CommunicationThread");
        communicationThread.start();
    }

    @Override
    public void stopRecognizing() {
        if (!activated) return;
        System.out.println("<-- Jetson Client | stopping communication thread... -->");
        this.activated = false;
        targetsList = new ArrayList<>();
        try {
            communicationThread.join(500);
        } catch (InterruptedException ignored) {}
        System.out.println("<-- Jetson Client | communication thread stopped -->");
    }

    @Override
    public void update() {
        this.targetsList = new ArrayList<>();
        if (!activated || results.equals("no-rst")) return;
        for (String target:results.split("/")) {
            String[] s = target.split(" ");
            if (s.length != 4) continue;
            final int id = Integer.parseInt(s[0]);
            final double x = Double.parseDouble(s[1]),
                    y = Double.parseDouble(s[2]),
                    area = Double.parseDouble(s[3]);
            // System.out.println(""<-- Jetson Client | updated target" + id + "at pixel (" +x + ", " + y  + ") -->");
            targetsList.add(new ObjectTargetRaw(id, x - cameraResolution[0]/2, y - cameraResolution[0]/2, area));
        }
    }

    @Override
    public List<ObjectTargetRaw> getRawTargets() {
        return targetsList;
    }

    public Thread.State getCommunicationThreadState() {
        return communicationThread == null ? null : communicationThread.getState();
    }
}
