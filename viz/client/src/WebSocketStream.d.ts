interface WebSocketStreamOptions {
  protocols: string | string[];
  signal: AbortSignal;
}

interface WebSocketStreamOpenedResult {
  extensions: string;
  protocol: string;
  readable: ReadableStream<Uint8Array>;
  writable: WritableStream<Uint8Array>;
}

interface WebSocketStreamClosedReason {
  /** 
   * @brief A number representing the closing code
   *  
   * @see https://developer.mozilla.org/en-US/docs/Web/API/CloseEvent/code#value list of CloseEvent codes.
   */
  closeCode: number;
  /** Must be shorter than 123 bytes */
  reason: string;
}

declare class WebSocketStream {
  constructor(url: string, options?: WebSocketStreamOptions);
  readonly url: string;

  /** @brief Fulfills once the socket connection is successfully opened. */
  readonly get opened(): Promise<WebSocketStreamOpenedResult>;
  
  /** @brief Fulfills once the socket connection is close. */
  readonly get closed(): Promise<WebSocketStreamClosedReason>;

  close(reason?: Partial<WebSocketStreamClosedReason>): void;
}
