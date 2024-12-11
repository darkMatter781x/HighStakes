declare global {
  interface Window {
    dash_clientside: unknown;
  }
}

export default function createCallback<Callback extends CallableFunction>(
  name: string,
  callback: Callback
): void {
  window.dash_clientside = Object.assign({}, window.dash_clientside, {
    clientside: Object.fromEntries([[name, callback]]),
  });
}
