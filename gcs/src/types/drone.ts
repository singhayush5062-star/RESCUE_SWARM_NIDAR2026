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
  lastUpdate: number | null;
}

export type ConnectionState = 'connecting' | 'connected' | 'disconnected' | 'error';
