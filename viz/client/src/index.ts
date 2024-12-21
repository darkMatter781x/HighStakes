import "./graph.css";
import { createPlot } from "./createPlot";

import { startPlotting } from "./callbacks/on_ws_message.js";

(async () => {
  const plot = await createPlot({ numOfLasers: 8 });
  await startPlotting(plot);
})();
