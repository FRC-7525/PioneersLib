package org.team7525.misc;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;

import java.lang.management.GarbageCollectorMXBean;
import java.lang.management.ManagementFactory;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.Consumer;
import java.util.function.Supplier;

public class Tracer {
  private static final class TraceStartData {
    private double startTime;
    private double totalGCTime;


    private void set(double startTime, double startGCTotalTime) {
      this.startTime = startTime;
      this.totalGCTime = startGCTotalTime;
    }
  }

  @SuppressWarnings("PMD.RedundantFieldInitializer")
  private static final class TracerState {
    private final NetworkTable rootTable;
    private final StringBuilder stackBuilder = new StringBuilder();
    private final ArrayList<String> traceStack = new ArrayList<>();

    private final ArrayList<String> traceStackHistory = new ArrayList<>();

    private final HashMap<String, Double> traceTimes = new HashMap<>();

    private final HashMap<String, TraceStartData> TRACE_START_TIMES = new HashMap<>();

    private final HashMap<String, TraceStartData> traceStartTimes = new HashMap<>();

    private final HashMap<String, DoublePublisher> publishers = new HashMap<>();

    boolean m_cyclePoisoned = false;

    boolean m_disabled = false;

    boolean m_disableNextCycle = false;

    int m_stackSize = 0;

    private final ArrayList<GarbageCollectorMXBean> m_gcs =
        new ArrayList<>(ManagementFactory.getGarbageCollectorMXBeans());
    private final DoublePublisher m_gcTimeEntry;
    private double m_gcTimeThisCycle = 0.0;

    private TracerState(String name, boolean threadLocalConstruction) {
      if (singleThreadedMode.get() && threadLocalConstruction) {
        DriverStation.reportError(
            "[Tracer] Tracer is in single threaded mode, cannot start traces on multiple threads",
            true);
        this.m_disabled = true;
      }
      anyTracesStarted.set(true);
      if (name == null) {
        this.rootTable = NetworkTableInstance.getDefault().getTable("Tracer");
      } else {
        this.rootTable = NetworkTableInstance.getDefault().getTable("Tracer").getSubTable(name);
      }
      this.m_gcTimeEntry = rootTable.getDoubleTopic("GCTime").publish();
    }

    private String appendTraceStack(String trace) {
      m_stackSize++;

      if (m_disabled) return "";

      traceStack.add(trace);

      if (!stackBuilder.isEmpty()) {
        stackBuilder.append("/");
      }

      stackBuilder.append(trace);

      String result = stackBuilder.toString();
      traceStackHistory.add(result);
      return result;
    }

    private String popTraceStack() {
      m_stackSize = Math.max(0, m_stackSize - 1);
      if (m_disabled) {
        return "";
      }
      if (traceStack.isEmpty() || traceStackHistory.isEmpty() || m_cyclePoisoned) {
        m_cyclePoisoned = true;
        return "";
      }
      traceStack.remove(traceStack.size() - 1);
      return traceStackHistory.remove(traceStackHistory.size() - 1);
    }

    private final List<GarbageCollectorMXBean> gcs = ManagementFactory.getGarbageCollectorMXBeans();
    private double cachedGC = 0.0;

    private double totalGCTime() {
      double gcTime = cachedGC;
      for (GarbageCollectorMXBean gc : gcs) {
        gcTime += gc.getCollectionTime();
      }
      return gcTime;
    }

    private void endCycle() {
      if (m_disabled != m_disableNextCycle || m_cyclePoisoned) {
        publishers.forEach((key, pub) -> pub.set(0.0));
      }

      if (!m_disabled) {
        for (var entry : traceTimes.entrySet()) {
          DoublePublisher publisher = publishers.computeIfAbsent(
                  entry.getKey(), key -> rootTable.getDoubleTopic(key).publish()
          );
          publisher.set(entry.getValue());
        }

        if (!gcs.isEmpty()) {
          m_gcTimeEntry.set(m_gcTimeThisCycle);
        }

        m_gcTimeThisCycle = 0.0;
      }

      traceTimes.clear();
      traceStackHistory.clear();
      m_disabled = m_disableNextCycle;
    }
  }

  private static final AtomicBoolean singleThreadedMode = new AtomicBoolean(false);
  private static final AtomicBoolean anyTracesStarted = new AtomicBoolean(false);
  private static final ThreadLocal<TracerState> threadLocalState =
      ThreadLocal.withInitial(
          () -> new TracerState(Thread.currentThread().getName(), true));

  private static void startTraceInner(final String name, final TracerState state) {
    String stack = state.appendTraceStack(name);
    if (state.m_disabled) return;

    TraceStartData data = state.traceStartTimes.computeIfAbsent(stack, k -> new TraceStartData());
    data.set(Timer.getFPGATimestamp() * 1_000.0, state.totalGCTime());
  }

  private static void endTraceInner(final TracerState state) {
    String stack = state.popTraceStack();
    if (!state.m_disabled) {
      if (stack.isEmpty()) {
        DriverStation.reportError(
            "[Tracer] Stack is empty,"
                + "this means that there are more endTrace calls than startTrace calls",
            true);
        return;
      }
      var startData = state.traceStartTimes.get(stack);
      double gcTimeSinceStart = state.totalGCTime() - startData.totalGCTime;
      state.m_gcTimeThisCycle += gcTimeSinceStart;
      state.traceTimes.put(
          stack, Timer.getFPGATimestamp() * 1_000.0 - startData.startTime - gcTimeSinceStart);
    }
    if (state.traceStack.isEmpty()) {
      state.endCycle();
    }
  }

  public static void startTrace(String name) {
    startTraceInner(name, threadLocalState.get());
  }

  public static void endTrace() {
    endTraceInner(threadLocalState.get());
  }

  public static void disableGcLoggingForCurrentThread() {
    TracerState state = threadLocalState.get();
    state.m_gcTimeEntry.close();
    state.m_gcs.clear();
  }

  public static void enableSingleThreadedMode() {
    if (anyTracesStarted.get()) {
      DriverStation.reportError(
          "[Tracer] Cannot enable single-threaded mode after traces have been started", true);
    } else {
      threadLocalState.set(new TracerState(null, false));
      singleThreadedMode.set(true);
    }
  }

  public static void disableTracingForCurrentThread() {
    final TracerState state = threadLocalState.get();
    state.m_disableNextCycle = true;
  }

  public static void enableTracingForCurrentThread() {
    final TracerState state = threadLocalState.get();
    state.m_disableNextCycle = false;
    if (state.m_stackSize == 0) {
      state.m_disabled = false;
    }
  }

  public static void traceFunc(String name, Runnable runnable) {
    final TracerState state = threadLocalState.get();
    startTraceInner(name, state);
    runnable.run();
    endTraceInner(state);
  }

  public static <R> R traceFunc(String name, Supplier<R> supplier) {
    final TracerState state = threadLocalState.get();
    startTraceInner(name, state);
    try {
      return supplier.get();
    } finally {
      endTraceInner(state);
    }
  }

  static void resetForTest() {
    threadLocalState.remove();
    singleThreadedMode.set(false);
    anyTracesStarted.set(false);
  }

  public static class SubstitutiveTracer {
    private final TracerState m_state;
    private final TracerState m_originalState;

    public SubstitutiveTracer(String name) {
      m_state = new TracerState(name, false);
      m_state.m_gcTimeEntry.close();
      m_state.m_gcs.clear();
      m_originalState = threadLocalState.get();
    }

    public void subIn() {
      threadLocalState.set(m_state);
    }

    public void subOut() {
      threadLocalState.set(m_originalState);
    }

    public void subWith(Runnable runnable) {
      subIn();
      try {
        runnable.run();
      } finally {
        subOut();
      }
    }
  }

  private static final long kMinPrintPeriod = 1000000;

  private long m_lastEpochsPrintTime;
  private long m_startTime;

  private final HashMap<String, Long> m_epochs = new HashMap<>();

  @Deprecated(since = "2025", forRemoval = true)
  public Tracer() {
    resetTimer();
  }

  @Deprecated(since = "2025", forRemoval = true)
  public void clearEpochs() {
    m_epochs.clear();
    resetTimer();
  }

  @Deprecated(since = "2025", forRemoval = true)
  public final void resetTimer() {
    m_startTime = RobotController.getFPGATime();
  }

  @Deprecated(since = "2025", forRemoval = true)
  public void addEpoch(String epochName) {
    long currentTime = RobotController.getFPGATime();
    m_epochs.put(epochName, currentTime - m_startTime);
    m_startTime = currentTime;
  }

  @Deprecated(since = "2025", forRemoval = true)
  public void printEpochs() {
    printEpochs(out -> DriverStation.reportWarning(out, false));
  }

  @Deprecated(since = "2025", forRemoval = true)
  public void printEpochs(Consumer<String> output) {
    long now = RobotController.getFPGATime();
    if (now - m_lastEpochsPrintTime > kMinPrintPeriod) {
      StringBuilder sb = new StringBuilder();
      m_lastEpochsPrintTime = now;
      m_epochs.forEach(
          (key, value) -> sb.append(String.format("\t%s: %.6fs\n", key, value / 1.0e6)));
      if (sb.length() > 0) {
        output.accept(sb.toString());
      }
    }
  }
}