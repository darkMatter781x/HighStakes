import type {
  Data as BadPlotlyData,
  BoxPlotData,
  PlotData,
  PlotlyHTMLElement,
} from "plotly.js-dist-min";
import { HomogenousTuple } from "../types/tuples";
// import createCallback from "./_createCallback";
import Plotly, { addTraces } from "plotly.js-dist-min";
import { read } from "fs";

type PlotlyData = BadPlotlyData extends infer D
  ? D extends Partial<PlotData>
    ? D & Partial<{ "marker.angle": number[]; "marker.angleref": number[] }>
    : D
  : never;

// If changed, also update the corresponding value in viz/host/app.py
const NUM_OF_LASERS = 2 as const;

const TRACE_ID = {
  PARTICLES: 0,
  CONTOUR: 1,
  CALCULATED_POSE: 2,
  ACTUAL_POSE: 3,
  RAW_ODOM_POSE: 4,
  // TODO: dynamically create traces for lasers
  LASERS: [5, 6, 7, 8, 9, 10, 11, 12],
} as const;

interface Point {
  x: number;
  y: number;
}

interface Pose extends Point {
  theta: number;
}

export type PlotlyExtendDataObject<TraceNum extends number> =
  PlotlyData extends infer D
    ? {
        updateData: { [K in keyof D]: HomogenousTuple<D[K], TraceNum> };
        traceIndices?: HomogenousTuple<number, TraceNum>;
        maxPoints?: number;
      }
    : never;

// Helper to prettify the type on hover
type Pretty<T> = {
  [K in keyof T]: T[K];
} & {};

// Get all possible keys from union type
type AllKeys<T> = T extends any ? keyof T : never;

// Create a union of all possible values for each key
type ValueOf<T, K extends AllKeys<T>> = T extends infer U
  ? K extends keyof U
    ? U[K]
    : never
  : never;

type PlotlyDataKeys = AllKeys<PlotlyData>;

type Merge<T> = {
  [K in AllKeys<T>]?: ValueOf<T, K>;
};

type MergedPlotlyData = Required<Merge<PlotlyData>>;

type PlotlyRestyleData = MergedPlotlyData extends infer D
  ? {
      [K in keyof D]?: D[K][] | D[K];
    }
  : never;

let graphDiv: PlotlyHTMLElement;

class Plot {
  private dataQueue: Record<number, PlotlyData> = {};

  /** Merges dataQueue into a single update command, and resets the dataQueue */
  update() {
    const data: PlotlyRestyleData = {};
    const keys: Set<PlotlyDataKeys> = new Set(
      Object.values(this.dataQueue).flatMap(
        (d) => Object.keys(d) as Array<PlotlyDataKeys>
      )
    );
    const traces: number[] = Object.keys(this.dataQueue).map(Number);

    for (const k of keys) {
      data[k] = [];
      for (const trace of traces) {
        const traceData = this.dataQueue[trace];
        if (k in traceData) {
          data[k].push(
            (
              traceData as typeof traceData extends Record<typeof k, any>
                ? typeof traceData
                : never
            )[k]
          );
        } else data[k].push(null);
      }
    }
    // console.log(data, traces);
    Plotly.restyle(graphDiv, data as PlotlyData, traces);

    this.dataQueue = {};
  }

  /** Adds data on to {@linkcode plotData} */
  addTrace(trace: number, data: PlotlyData) {
    this.dataQueue[trace] = data;
  }
}

const plot: Plot = new Plot();

function handle_particles(particles: Record<"x" | "y" | "weight", number[]>) {
  // return null;
  plot.addTrace(TRACE_ID.CONTOUR, {
    x: particles.x,
    y: particles.y,
    z: particles.weight,
  });
  plot.addTrace(TRACE_ID.PARTICLES, {
    x: particles.x,
    y: particles.y,
    "marker.opacity": particles.weight.map(
      (w) => w / Math.max(...particles.weight)
    ),
  });
  // return await plot<2>({
  //   updateData: {
  //     x: [particles.x, particles.x],
  //     y: [particles.y, particles.y],
  //     z: [[], particles.weight],
  //     "marker.opacity": [
  //       particles.weight.map((w) => w / Math.max(...particles.weight)),
  //       [],
  //     ],
  //   },
  //   traceIndices: [TRACE_ID.PARTICLES, TRACE_ID.CONTOUR],
  //   maxPoints: particles.x.length,
  // });
}

function handle_lasers([id, start, end]: [number, Point, Point]) {
  plot.addTrace(TRACE_ID.LASERS[id], {
    x: [start.x, end.x],
    y: [start.y, end.y],
  });

  // <2>({
  //   updateData: {
  //     x: Object.values(lasers).map(([a, b]) => [a.x, b.x]) as [
  //       [number, number],
  //       [number, number]
  //     ],
  //     y: Object.values(lasers).map(([a, b]) => [a.y, b.y]) as [
  //       [number, number],
  //       [number, number]
  //     ],
  //   },
  //   traceIndices: Object.keys(lasers).map(
  //     (k) => TRACE_ID.LASERS[Number.parseFloat(k)]
  //   ) as [number, number],
  //   maxPoints: 2,
  // });
}

// let callsToHandlePose = 0;
// /** trace to pose map */
// const poses: Record<number, Pose> = {};
function handle_pose(pose: Pose, trace: number) {
  plot.addTrace(trace, {
    x: [pose.x],
    y: [pose.y],
    "marker.angle": [(Math.PI / 2 - pose.theta) * (180 / Math.PI)],
  });
}

let calculatedPose: Pose = { x: 0, y: 0, theta: 0 };
let actualPose: Pose = { x: 0, y: 0, theta: 0 };
let rawPose: Pose = { x: 0, y: 0, theta: 0 };
let errorElement: HTMLElement | null = null;

/** Maps event types to the corresponding handler */
const handlers = {
  particles: handle_particles,
  lasers: handle_lasers,
  calculated_pose: (pose: Pose) => {
    calculatedPose = pose;
    handle_pose(pose, TRACE_ID.CALCULATED_POSE);
  },
  actual_pose: (pose: Pose) => {
    actualPose = pose;
    handle_pose(pose, TRACE_ID.ACTUAL_POSE);
    if (errorElement == null) return;
    errorElement.innerText = `ERROR:\t\tCalc\t\tRaw
\t\tx:\t${(actualPose.x - calculatedPose.x).toFixed(3)}in\t${(
      actualPose.x - rawPose.x
    ).toFixed(3)}in
\t\ty:\t${(actualPose.y - calculatedPose.y).toFixed(3)}in\t${(
      actualPose.y - rawPose.y
    ).toFixed(3)}in`;
  },
  raw_odom_pose: (pose: Pose) => {
    rawPose = pose;
    handle_pose(pose, TRACE_ID.RAW_ODOM_POSE);
  },
};

class TimingHandler {
  public Timer = ((handler: TimingHandler) =>
    class Timer {
      private readonly start: number = performance.now();
      constructor(private readonly label: string) {}

      end() {
        handler.recordTime(this.label, this.start, performance.now());
      }
    })(this);
  constructor(
    private readonly output: HTMLElement,
    resetButton: HTMLButtonElement
  ) {
    const timingHandler = this;
    resetButton.addEventListener("click", () => {
      timingHandler.timeSpent = {};
      timingHandler.startTime = {};
    });
  }
  public reportTimings() {
    let text = "";
    const now = performance.now();
    for (const label in this.timeSpent) {
      const timePct =
        (this.timeSpent[label] / (now - this.startTime[label])) * 100;
      text += `${label}: ${timePct.toFixed(2)}%\t ${this.avgDuration[
        label
      ].toFixed(2)}ms\n`;
    }
    this.output.innerHTML = text;
  }

  private recordTime(label: string, startTime: number, endTime: number) {
    const duration = endTime - startTime;
    if (!(label in this.timeSpent)) {
      this.avgDuration[label] = duration;
      this.timeSpent[label] = duration;
      this.startTime[label] = startTime;
    } else {
      this.avgDuration[label] *= 0.9;
      this.avgDuration[label] += 0.1 * duration;
      this.timeSpent[label] += endTime - startTime;
    }
  }

  private avgDuration: Record<string, number> = {};
  private timeSpent: Record<string, number> = {};
  private startTime: Record<string, number> = {};
}

export async function startPlotting(plotlyDiv: PlotlyHTMLElement) {
  console.log("Start");
  graphDiv = plotlyDiv;
  const wss = new WebSocketStream("ws://localhost:8766/data");
  const {
    readable,
    writable: _,
    extensions: _0,
    protocol: _1,
  } = await wss.opened;

  wss.closed.then((reason) => {
    alert(`Connection closed: ${reason.reason}`);
  });

  const timingOutput = document.getElementById("timing-output");
  if (!timingOutput) throw new Error("Could not find timing output element!");
  errorElement = document.getElementById("error");
  if (!errorElement) throw new Error("Could not find error output element!");

  const timings = new TimingHandler(
    timingOutput,
    document.getElementById("reset-timings") as HTMLButtonElement
  );
  const dataHandler = async function (msg: unknown) {
    if (typeof msg != "string")
      throw new Error("Expected websocket message to be string!");

    const data = JSON.parse(msg);
    for (const _eventType in handlers) {
      const eventType = _eventType as keyof typeof handlers;
      if (eventType in data) {
        const timer = new timings.Timer(`handle_event`);
        const eventSpecificTimer = new timings.Timer(`handle_${eventType}`);
        await handlers[eventType](data[eventType]);
        // console.log(
        //   `Time taken to handle ${eventType}: ${performance.now() - start}ms`
        // );
        eventSpecificTimer.end();
        timer.end();
        // TODO: Try running after previous plot update is complete
        if (eventType == "calculated_pose") {
          setTimeout(() => {
            const plotTimer = new timings.Timer("plot");
            plot.update();
            plotTimer.end();
          }, 0);
        }
      }
    }
  };

  (async () => {
    const reader = readable.getReader();
    while (true) {
      const timer = new timings.Timer("read_message");
      const { value, done } = await reader.read();
      timer.end();
      timings.reportTimings();
      if (done) {
        break;
      }
      await dataHandler(value);
    }
  })();
}

// createCallback("on_websocket_message", function (msg: { data: string }) {
//   if (!msg) {
//     return null;
//   } // no data, just return
//   const data = JSON.parse(msg.data);
//   for (const _eventType in handlers) {
//     const eventType = _eventType as keyof typeof handlers;
//     if (eventType in data) {
//       console.log(eventType);
//       return handlers[eventType](data[eventType]);
//     }
//   }
// });
