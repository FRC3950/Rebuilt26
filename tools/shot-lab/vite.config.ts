import { defineConfig } from "vite";
export default defineConfig({
  build: {
    rollupOptions: {
      input: {
        index: "index.html",
        advanced: "advanced.html",
        map: "map.html",
      },
    },
  },
});
