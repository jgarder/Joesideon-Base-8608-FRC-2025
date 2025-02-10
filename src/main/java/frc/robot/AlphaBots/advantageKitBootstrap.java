package frc.robot.AlphaBots;

import java.util.HashMap;
import java.util.Map;
import java.util.function.BiConsumer;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggedPowerDistribution;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Robot;

public class advantageKitBootstrap {
    public final Robot robot;
    public boolean loggingactive = false;
    public advantageKitBootstrap(Robot robot2) {
        robot = robot2;
        startAdvantageKitLogger();//run before robot container boots. //declare in robot as a private final?

        //StartWpiLogger();
    }

    public void startAdvantageKitLogger() {
      if(loggingactive){return;}else{loggingactive = true;}
      Logger.recordMetadata("ProjectName", "MantaJoe"); // Set a metadata value

      if (robot.isReal()) {
          Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
          Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
          //new PowerDistribution(1, ModuleType.kRev); // Enables power distribution logging
          LoggedPowerDistribution.getInstance(1, ModuleType.kRev);
      } else {
          robot.setUseTiming(false); // Run as fast as possible
          String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
          Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
          Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
      }

      

      // Log active commands
      Map<String, Integer> commandCounts = new HashMap<>();
      BiConsumer<Command, Boolean> logCommandFunction =
          (Command command, Boolean active) -> {
            String name = command.getName();
            int count = commandCounts.getOrDefault(name, 0) + (active ? 1 : -1);
            commandCounts.put(name, count);
            Logger.recordOutput(
                "CommandsUnique/" + name + "_" + Integer.toHexString(command.hashCode()), active);
            Logger.recordOutput("CommandsAll/" + name, count > 0);
          };
      CommandScheduler.getInstance()
          .onCommandInitialize(
              (Command command) -> {
                logCommandFunction.accept(command, true);
              });
      CommandScheduler.getInstance()
          .onCommandFinish(
              (Command command) -> {
                logCommandFunction.accept(command, false);
              });
      CommandScheduler.getInstance()
          .onCommandInterrupt(
              (Command command) -> {
                logCommandFunction.accept(command, false);
              });

      // Logger.disableDeterministicTimestamps() // See "Deterministic Timestamps" in the "Understanding Data Flow" page
      Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may be added.
  }

  // private void StartWpiLogger() {
  //   //wpilogger test
  //   // Set out log file to be in its own folder
  //   if (Robot.isSimulation()) {
  //     DataLogManager.start("src/main");
  //   } else {
  //    DataLogManager.start();
  //   }
  //   // Log data that is being put to shuffleboard
  //  DataLogManager.logNetworkTables(true);
  //   // Log the DS data and joysticks (doesnt work if advantage kit is active)
  //   DriverStation.startDataLog(DataLogManager.getLog(), true);
  // }

}
