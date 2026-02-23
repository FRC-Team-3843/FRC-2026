package frc.robot.util;

import com.sun.net.httpserver.HttpExchange;
import com.sun.net.httpserver.HttpServer;
import edu.wpi.first.wpilibj.DriverStation;
import java.io.File;
import java.io.IOException;
import java.io.OutputStream;
import java.net.InetSocketAddress;
import java.nio.file.Files;

/**
 * Lightweight HTTP server that serves static files from the deploy directory.
 *
 * <p>Used to serve Elastic Dashboard layout JSON so the dashboard can load layouts
 * directly from the robot via Ctrl+D (fetch from robot).
 */
public final class WebServer {
  private static HttpServer server;

  private WebServer() {}

  /**
   * Start serving the given directory on the specified port.
   *
   * @param port the HTTP port (typically 5800)
   * @param rootPath absolute path to the directory to serve
   */
  public static void start(int port, String rootPath) {
    if (server != null) {
      return; // Already started
    }
    try {
      server = HttpServer.create(new InetSocketAddress(port), 0);
      File root = new File(rootPath);
      server.createContext("/", exchange -> handleRequest(exchange, root));
      server.setExecutor(null); // Default single-thread executor
      server.start();
    } catch (IOException e) {
      DriverStation.reportError("WebServer failed to start on port " + port + ": " + e.getMessage(), false);
    }
  }

  private static void handleRequest(HttpExchange exchange, File root) throws IOException {
    String path = exchange.getRequestURI().getPath();
    if (path.equals("/")) {
      path = "/index.html";
    }
    File file = new File(root, path);

    if (!file.exists() || !file.isFile() || !file.getCanonicalPath().startsWith(root.getCanonicalPath())) {
      byte[] resp = "404 Not Found".getBytes();
      exchange.sendResponseHeaders(404, resp.length);
      try (OutputStream os = exchange.getResponseBody()) {
        os.write(resp);
      }
      return;
    }

    byte[] data = Files.readAllBytes(file.toPath());
    String contentType = guessContentType(file.getName());
    exchange.getResponseHeaders().set("Content-Type", contentType);
    exchange.getResponseHeaders().set("Access-Control-Allow-Origin", "*");
    exchange.sendResponseHeaders(200, data.length);
    try (OutputStream os = exchange.getResponseBody()) {
      os.write(data);
    }
  }

  private static String guessContentType(String name) {
    if (name.endsWith(".json")) return "application/json";
    if (name.endsWith(".html")) return "text/html";
    if (name.endsWith(".js")) return "application/javascript";
    if (name.endsWith(".css")) return "text/css";
    if (name.endsWith(".png")) return "image/png";
    return "application/octet-stream";
  }
}
