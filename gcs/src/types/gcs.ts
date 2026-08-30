export type ExecutionMode = 'SIMULATION' | 'HARDWARE';

export type ActiveTab = 'CONTROL' | 'PLANNING' | 'MANUAL' | 'VIDEO' | 'LOGS';

export interface LogEntry {
  id: string;
  /** Display string, already localised. */
  timestamp: string;
  /** Epoch milliseconds, for ordering. Sorting on `timestamp` looks like it
   * works and then silently breaks on any 12-hour locale, where "9:59:00 PM"
   * sorts before "10:00:00 AM". Merging two log sources needs a real key. */
  sortKey: number;
  level: 'INFO' | 'WARN' | 'ERROR' | 'CMD';
  source: string; // e.g. 'drone0', 'GCS', 'SwarmManager'
  message: string;
}

export interface DetailedDroneTelemetry {
  namespace: string;
  connected: boolean;
  mode: string; // e.g. 'OFFBOARD', 'AUTO_MISSION', 'GUIDED', 'RTL', 'LAND', 'STABILIZED'
  armed: boolean;
  battery: {
    percentage: number;
    voltage: number;
  };
  gps: {
    lat: number;
    lon: number;
    alt: number;
    satellites: number;
  };
  heading: number; // degrees 0-360
  speed: number; // m/s
  rssi: number; // dBm e.g. -65
  ekfOk: boolean;
  lastUpdate: number;
}
