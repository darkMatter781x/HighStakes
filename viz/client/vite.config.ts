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
      },
      sourcemap: DEV_MODE,
      minify: !DEV_MODE,
      rollupOptions: {
        output: {
          compact: !DEV_MODE,
          
        },
      },
    },
  } satisfies UserConfig;
  return config;
});
