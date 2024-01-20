import java.io.BufferedReader;
import java.io.InputStreamReader;
import java.net.ConnectException;
import java.net.HttpURLConnection;
import java.net.SocketTimeoutException;
import java.net.URL;

public class TestJetsonVisionClient {
    public static void main(String[] args) {
        String serverUrl = "http://localhost:8888/results";

        while (true) {
            try {
                // Create URL object
                URL url = new URL(serverUrl);

                // Open connection
                HttpURLConnection connection = (HttpURLConnection) url.openConnection();

                // Set timeout (adjust as needed)
                connection.setConnectTimeout(500);

                // Set request method
                connection.setRequestMethod("GET");

                try {
                    // Get response code
                    int responseCode = connection.getResponseCode();

                    // Check if the request was successful
                    if (responseCode == HttpURLConnection.HTTP_OK) {
                        // Read the response
                        BufferedReader reader = new BufferedReader(new InputStreamReader(connection.getInputStream()));
                        String line;
                        while ((line = reader.readLine()) != null) {
                            // Print each line from the server
                            System.out.println(line);
                        }
                        reader.close();
                    } else {
                        // Print an error message if the request was not successful
                        System.out.println("Error: " + responseCode);
                    }
                } catch (SocketTimeoutException e) {
                    // Handle timeout (connection not established yet)
                    System.out.println("Connection timeout. Waiting for the server to start...");
                } catch(ConnectException e) {
                    System.out.println("Connection timeout. Waiting for the server to start...");
                }

                // Close the connection
                connection.disconnect();

                // Sleep for a while before making the next request
                Thread.sleep(500); // Adjust the sleep interval as needed
            } catch (Exception e) {
                // Handle other exceptions
                e.printStackTrace();
            }
        }
    }
}
