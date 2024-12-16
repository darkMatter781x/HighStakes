import Plotly, {
  Data,
  ScatterData as OldScatterData,
  PlotData,
  PlotlyHTMLElement,
} from "plotly.js-dist-min";

export interface PlotConfig {
  numOfLasers: number;
}

interface ScatterData extends OldScatterData {
  "marker.angle": number[] | number;
  marker: OldScatterData["marker"] & Partial<{ angle: number[] | number }>;
}

export async function createPlot(config: PlotConfig): Promise<PlotlyHTMLElement> {
  const graphDiv = document.getElementById("graph");
  if (graphDiv === null) {
    throw "Could not find graph div";
  }

  const particleData = {
    x: [0],
    y: [0],
    z: [],
    mode: "markers",
    type: "scattergl",
    marker: {
      opacity: [1],
    }
  } satisfies Partial<ScatterData>;

  const contourData = {
    x: [0],
    y: [0],
    z: [0],
    name: "density",
    ncontours: 10,
    colorscale: [
      [0, "rgba(0,0,255,1)"],
      [1, "rgba(255,255,255,0)"],
    ],
    reversescale: true,
    showscale: false,
    type: "histogram2dcontour",
    histfunc: "sum",
    // colorbar: {
    //   x: 1,
    //   y: 0,
    //   len: 2 / 3,
    //   yanchor: "bottom",
    // },
    showlegend: false,
    marker: { opacity: [] },
  } satisfies Partial<PlotData>;

  const laserDataTemplate = {
    x: [0, 0],
    y: [0, 0],
    mode: "lines+markers",
    type: "scattergl",
  } satisfies Partial<ScatterData>;

  const laserDatas = Array(config.numOfLasers)
    .fill(null)
    .map((_, i) => {
      const out: Partial<ScatterData> = JSON.parse(
        JSON.stringify(laserDataTemplate)
      ) as typeof laserDataTemplate;

      out.name = `Laser ${i}`;
      return out;
    });

  const poseDataTemplate = {
    x: [0],
    y: [0],
    mode: "markers",
    type: "scatter",
    marker: {
      symbol: "triangle-up",
      angle: [0],
      size: 15,
    },
  } satisfies Partial<ScatterData>;

  const poseDatas = ["Actual", "Calculated"].map((name) => {
    const out: Partial<ScatterData> = JSON.parse(
      JSON.stringify(poseDataTemplate)
    ) as typeof poseDataTemplate;

    out.name = name;
    return out;
  });

  const data = [
    particleData,
    contourData,
    ...poseDatas,
    ...laserDatas,
  ] satisfies Data[];
  console.log("data:", JSON.parse(
    JSON.stringify(data)
  ));
  const layout = {
    yaxis: { scaleanchor: "x", scaleratio: 1, range: [-72, 72] },
    xaxis: { range: [-72, 72] },
    // showlegend: false,
    legend: { x: 1, y: 1 },
  } satisfies Partial<Plotly.Layout>;

  return await Plotly.newPlot(graphDiv, data, layout);
}
