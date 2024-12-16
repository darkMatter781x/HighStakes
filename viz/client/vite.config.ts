import { resolve } from "path";
import { defineConfig, type UserConfig } from "vite";

export default defineConfig(({ mode }): UserConfig => {
  const DEV_MODE = mode.toLowerCase() === "dev";
  const config = {
    build: {
      lib: {
        entry: resolve(__dirname, "src/index.ts"),
        name: "VizClient",
        // the proper extensions will be added
        fileName: "viz-client",
        formats: ["umd"],
      },
      sourcemap: DEV_MODE
    }
  } satisfies UserConfig;
  return config;
});
