export interface GpsFix {
  lat: number;
  lon: number;
  alt: number;
  stamp: number;
}

export interface BatteryStatus {
  percentage: number;
  voltage: number;
}

export interface DroneTelemetry {
  namespace: string;
  connected: boolean;
  gps: GpsFix | null;
  battery: BatteryStatus | null;
  /** Ground speed in m/s, from /<ns>/self_localization/twist.
   *
   * `null` means "nothing has published a velocity for this drone yet",
   * which is NOT the same as "it is stationary". The SPEED readout used to
   * come from an optional field no hook ever set, so it defaulted to 0.0 and
   * every drone read "0.0 m/s" for the whole mission -- indistinguishable
   * from a genuine measurement of a hovering swarm. */
  speed: number | null;
  /** Climb rate in m/s, positive up. Same null semantics as `speed`. */
  verticalSpeed: number | null;
  lastUpdate: number | null;
}

export type ConnectionState = 'connecting' | 'connected' | 'disconnected' | 'error';
