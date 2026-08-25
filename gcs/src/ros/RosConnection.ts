import * as ROSLIB from 'roslib';
import type { ConnectionState } from '../types/drone';

export type ConnectionListener = (state: ConnectionState) => void;

/**
 * Single shared rosbridge WebSocket connection for the whole GCS.
 * The mission brief requires all drones to report through one operator
 * interface, so one connection (not one per drone) is the right shape.
 */
class RosConnection {
  private ros: ROSLIB.Ros;
  private listeners = new Set<ConnectionListener>();
  private state: ConnectionState = 'connecting';
  private url: string;

  constructor(url: string) {
    this.url = url;
    this.ros = new ROSLIB.Ros({});
    this.ros.on('connection', () => this.setState('connected'));
    this.ros.on('close', () => this.setState('disconnected'));
    this.ros.on('error', () => this.setState('error'));
    this.connect();
  }

  connect() {
    this.setState('connecting');
    this.ros.connect(this.url);
  }

  private setState(state: ConnectionState) {
    this.state = state;
    this.listeners.forEach((l) => l(state));
  }

  getState() {
    return this.state;
  }

  onStateChange(listener: ConnectionListener) {
    this.listeners.add(listener);
    listener(this.state);
    return () => this.listeners.delete(listener);
  }

  getRos() {
    return this.ros;
  }
}

// rosbridge_server default port. Override with VITE_ROSBRIDGE_URL if the GCS
// runs on a different machine than rosbridge (e.g. GCS laptop <-> radio link
// <-> companion computer running rosbridge_server).
const ROSBRIDGE_URL = import.meta.env.VITE_ROSBRIDGE_URL ?? 'ws://localhost:9090';

export const rosConnection = new RosConnection(ROSBRIDGE_URL);
