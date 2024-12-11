import type { Data, extendTraces } from "plotly.js";
import { HomogenousTuple } from "../types/tuples";

type Tail<T extends unknown[]> = T extends [unknown, ...infer Rest]
  ? Rest
  : never;

const trace_ids = {
  particles: 0,
  contour: 1,
} as const;

export type DashExtendDataObject<TraceNum extends number> = Data extends infer D
  ? {
      updateData: { [K in keyof D]: HomogenousTuple<D[K], TraceNum> };
      traceIndices?: HomogenousTuple<number, TraceNum>;
      maxPoints?: number;
    }
  : never;

function extendData<N extends number>(data: DashExtendDataObject<N>) {
  return Object.values(data);
}

function handle_particles(particles: Record<"x" | "y" | "weight", number[]>) {
  return extendData<2>({
    updateData: {
      x: [particles.x, particles.x],
      y: [particles.y, particles.y],
      z: [[], particles.weight],
      "marker.opacity": [
        particles.weight.map((w) => w / Math.max(...particles.weight)),
        [],
      ],
    },
    traceIndices: [trace_ids.particles, trace_ids.contour],
    maxPoints: particles.x.length,
  });
}

/** Maps event types to the corresponding handler */
const handlers = {
  particles: handle_particles,
};

// @ts-ignore
window.dash_clientside = Object.assign({}, window.dash_clientside, {
  clientside: {
    on_websocket_message: function (msg: { data: string }) {
      if (!msg) {
        return null;
      } // no data, just return
      const data = JSON.parse(msg.data);
      for (const _eventType in handlers) {
        const eventType = _eventType as keyof typeof handlers;
        if (eventType in data) return handlers[eventType](data[eventType]);
      }
    },
  },
});
