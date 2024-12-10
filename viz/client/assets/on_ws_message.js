const trace_ids = {
  particles: 0,
  contour: 1,
};

function handle_particles(particles) {
  return [
    {
      x: [particles.x, particles.x],
      y: [particles.y, particles.y],
      z: [[], particles.weight],
      "marker.opacity": [
        particles.weight.map((w) => w / Math.max(...particles.weight)),
        [],
      ],
    },
    [trace_ids.particles, trace_ids.contour],
    particles.x.length,
  ];
}
/** Maps event types to the corresponding handler */
const handlers = {
  particles: handle_particles,
};

window.dash_clientside = Object.assign({}, window.dash_clientside, {
  clientside: {
    on_websocket_message: function (msg) {
      if (!msg) {
        return null;
      } // no data, just return
      const data = JSON.parse(msg.data);
      for (const eventType in handlers)
        if (eventType in data) return handlers[eventType](data[eventType]);
    },
  },
});
