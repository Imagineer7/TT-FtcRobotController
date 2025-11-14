package org.firstinspires.ftc.teamcode.webinterface;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.robotcore.internal.webserver.WebHandler;

import java.lang.reflect.Method;

/**
 * Fallback OpMode to register style endpoints if annotation-based registration didn't occur.
 * Run this OpMode once (INIT then START) and then visit:
 *   http://<rc-ip>:8080/improved-styles.css
 *   http://<rc-ip>:8080/custom/style-injector
 * On emulator w/ forward (example): http://localhost:8082/improved-styles.css
 */
@TeleOp(name = "Register Web Styles", group = "Utilities")
public class RegisterWebStylesOpMode extends OpMode {
    private boolean attempted;
    private boolean success;
    private String errorMessage = "";

    @Override
    public void init() {
        register();
        telemetry.addData("Web Style Registration", success ? "SUCCESS" : "FAILED");
        if (!success) telemetry.addData("Error", errorMessage);
        telemetry.addData("Test Endpoint", "/custom/teststyle");
        telemetry.addData("CSS Endpoint", "/improved-styles.css");
        telemetry.addData("Injector Page", "/custom/style-injector");
        telemetry.addData("Note", "Access via RC IP or forwarded localhost port");
        telemetry.update();
    }

    @Override
    public void loop() {
        // No continuous action needed
    }

    private void register() {
        if (attempted) return;
        attempted = true;
        try {
            Object activity = AppUtil.getInstance().getActivity();
            if (activity == null) throw new IllegalStateException("Activity not yet available");

            // Try getService() method reflectively (may be private)
            Method getService = null;
            try { getService = activity.getClass().getDeclaredMethod("getService"); } catch (NoSuchMethodException ignored) {}
            if (getService == null) throw new NoSuchMethodException("getService() not found on RC Activity");
            getService.setAccessible(true);
            Object service = getService.invoke(activity);
            if (service == null) throw new IllegalStateException("Robot Controller service null");

            // getWebServer()
            Method getWebServer = service.getClass().getMethod("getWebServer");
            Object webServer = getWebServer.invoke(service);
            if (webServer == null) throw new IllegalStateException("WebServer null");

            // getWebHandlerManager()
            Method getMgr = webServer.getClass().getMethod("getWebHandlerManager");
            Object manager = getMgr.invoke(webServer);
            if (manager == null) throw new IllegalStateException("WebHandlerManager null");

            // register(String, WebHandler)
            Method register = manager.getClass().getMethod("register", String.class, WebHandler.class);

            register.invoke(manager, "/improved-styles.css", new ImprovedWebStyling.ImprovedStylesHandler());
            register.invoke(manager, "/custom/style-injector", new ImprovedWebStyling.StyleInjectorPageHandler());
            register.invoke(manager, "/custom/teststyle", new ImprovedWebStyling.SimpleTestHandler());

            RobotLog.ii("RegisterWebStyles", "Registered style endpoints via fallback OpMode");
            success = true;
        } catch (Exception e) {
            success = false;
            errorMessage = e.getClass().getSimpleName() + ": " + (e.getMessage() == null ? "(no message)" : e.getMessage());
            RobotLog.ee("RegisterWebStyles", "Failed fallback style registration: " + e);
        }
    }
}
