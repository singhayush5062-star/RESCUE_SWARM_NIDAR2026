import { useState, useEffect, useRef } from 'react';
import type { DroneDetectionState } from '../ros/useDetections';
import type { ExecutionMode } from '../types/gcs';
import type { DroneTelemetry } from '../types/drone';
import './VideoPanel.css';

interface VideoPanelProps {
  droneNamespaces: string[];
  frames: Record<string, string>;
  byDrone: Record<string, DroneDetectionState>;
  total: number;
  observations: number;
  executionMode?: ExecutionMode;
  /** Live telemetry, so the HUD shows this drone's real altitude and
   * position instead of the fixed numbers it used to print. */
  drones?: DroneTelemetry[];
}

export function VideoPanel({
  droneNamespaces,
  frames,
  byDrone,
  total,
  observations,
  executionMode = 'SIMULATION',
  drones = [],
}: VideoPanelProps) {
  const [expandedNs, setExpandedNs] = useState<string | null>(null);
  const [cameraMode, setCameraMode] = useState<'THERMAL' | 'EO_RGB'>('THERMAL');
  const telemetryByNs = Object.fromEntries(drones.map((d) => [d.namespace, d]));
  // The camera-mode buttons restyle the SYNTHETIC canvas only -- there is one
  // real camera per drone and no thermal sensor in the sim or on the
  // airframe. Offering the toggle over a live feed implies a sensor switch
  // that does not happen, so it only appears while feeds are synthetic.
  const anyRealFeed = droneNamespaces.some((ns) => !!frames[ns]);

  return (
    <div className="video-grid-container">
      <div className="video-grid-header">
        <div style={{ display: 'flex', alignItems: 'center', gap: 10 }}>
          <span className="icon" style={{ color: 'var(--primary-bright)' }}>
            videocam
          </span>
          <span className="obsidian-card-header" style={{ border: 'none', marginBottom: 0, padding: 0 }}>
            LIVE VIDEO FEEDS & DETECTIONS ({executionMode})
          </span>
        </div>

        <div style={{ display: 'flex', alignItems: 'center', gap: 12 }}>
          <div className="obsidian-badge badge-warning" style={{ fontSize: 11 }}>
            {total} PERSONS TRACKED ({observations} DETECTIONS)
          </div>

          {anyRealFeed ? (
            <div className="obsidian-badge badge-success" style={{ fontSize: 11 }}>
              LIVE CAMERA
            </div>
          ) : (
            <div style={{ display: 'flex', border: '1px solid var(--border-mid)' }}
                 title="Styles the placeholder shown while no camera feed is arriving">
              <button
                className={`log-filter-btn ${cameraMode === 'THERMAL' ? 'active' : ''}`}
                onClick={() => setCameraMode('THERMAL')}
              >
                THERMAL IR
              </button>
              <button
                className={`log-filter-btn ${cameraMode === 'EO_RGB' ? 'active' : ''}`}
                onClick={() => setCameraMode('EO_RGB')}
              >
                EO / RGB
              </button>
            </div>
          )}
        </div>
      </div>

      <div className="video-grid-4up">
        {droneNamespaces.map((ns) => {
          const frame = frames[ns];
          const detState = byDrone[ns];
          const hasDetection = detState && detState.trackIds.size > 0;

          return (
            <div
              key={ns}
              className="video-feed-card"
              onClick={() => setExpandedNs(ns)}
              style={{ cursor: 'pointer' }}
            >
              {frame ? (
                <img
                  src={frame}
                  alt={`${ns} video feed`}
                  style={{ width: '100%', height: '100%', objectFit: 'cover' }}
                />
              ) : (
                <SyntheticFeedCanvas
                  droneNs={ns}
                  cameraMode={cameraMode}
                  hasDetection={hasDetection}
                />
              )}

              {/* HUD Overlay */}
              <div className="video-hud-overlay">
                <div className="hud-top-bar">
                  <span style={{ fontWeight: 700, color: 'var(--primary-bright)' }}>
                    {ns.toUpperCase()}
                  </span>
                  <span style={{ color: hasDetection ? 'var(--status-warning)' : 'var(--status-success)' }}>
                    {hasDetection
                      ? `TARGET DETECTED (${detState.trackIds.size})`
                      : 'SCANNING...'}
                  </span>
                  <span>{cameraMode}</span>
                </div>

                <div className="hud-crosshair" />

                <div className="hud-top-bar">
                  <span>ALT: {telemetryByNs[ns]?.gps
                    ? `${telemetryByNs[ns].gps!.alt.toFixed(1)}m` : '—'}</span>
                  <span>{frame ? 'LIVE' : 'NO SIGNAL'}</span>
                  <span>LAT: {telemetryByNs[ns]?.gps
                    ? `${telemetryByNs[ns].gps!.lat.toFixed(5)}°` : '—'}</span>
                </div>
              </div>
            </div>
          );
        })}
      </div>

      {/* Expanded Modal View */}
      {expandedNs && (
        <div
          className="modal-backdrop"
          onClick={() => setExpandedNs(null)}
          style={{ zIndex: 1000 }}
        >
          <div
            className="modal-card"
            style={{ width: '85vw', height: '80vh', maxWidth: 1100 }}
            onClick={(e) => e.stopPropagation()}
          >
            <div className="modal-header">
              <div className="modal-title">
                <span className="icon">fullscreen</span>
                ENLARGED FEED - {expandedNs.toUpperCase()}
              </div>
              <button className="obsidian-btn" onClick={() => setExpandedNs(null)}>
                ✕
              </button>
            </div>
            <div className="modal-body" style={{ flex: 1, padding: 0, position: 'relative' }}>
              {frames[expandedNs] ? (
                <img
                  src={frames[expandedNs]}
                  alt={`${expandedNs} stream`}
                  style={{ width: '100%', height: '100%', objectFit: 'contain' }}
                />
              ) : (
                <SyntheticFeedCanvas
                  droneNs={expandedNs}
                  cameraMode={cameraMode}
                  hasDetection={!!(byDrone[expandedNs] && byDrone[expandedNs].trackIds.size > 0)}
                />
              )}
            </div>
          </div>
        </div>
      )}
    </div>
  );
}

interface SyntheticCanvasProps {
  droneNs: string;
  cameraMode: 'THERMAL' | 'EO_RGB';
  hasDetection?: boolean;
}

function SyntheticFeedCanvas({ droneNs, cameraMode, hasDetection }: SyntheticCanvasProps) {
  const canvasRef = useRef<HTMLCanvasElement>(null);

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;
    const ctx = canvas.getContext('2d');
    if (!ctx) return;

    let animId: number;
    let tick = 0;

    const render = () => {
      tick++;
      const w = canvas.width;
      const h = canvas.height;

      if (cameraMode === 'THERMAL') {
        ctx.fillStyle = '#061018';
        ctx.fillRect(0, 0, w, h);

        ctx.strokeStyle = '#0e2330';
        ctx.lineWidth = 1;
        for (let i = 0; i < h; i += 30) {
          ctx.beginPath();
          ctx.moveTo(0, i);
          ctx.lineTo(w, i);
          ctx.stroke();
        }
      } else {
        ctx.fillStyle = '#111a14';
        ctx.fillRect(0, 0, w, h);

        ctx.strokeStyle = '#1a2b1f';
        ctx.lineWidth = 1;
        for (let i = 0; i < h; i += 30) {
          ctx.beginPath();
          ctx.moveTo(0, i);
          ctx.lineTo(w, i);
          ctx.stroke();
        }
      }

      const targetX = (w / 2) + Math.sin(tick * 0.03) * 40;
      const targetY = (h / 2) + Math.cos(tick * 0.02) * 25;

      if (hasDetection || droneNs === 'drone1' || droneNs === 'drone2') {
        const rad = ctx.createRadialGradient(targetX, targetY, 2, targetX, targetY, 22);
        rad.addColorStop(0, '#ff9900');
        rad.addColorStop(0.5, 'rgba(255, 60, 0, 0.4)');
        rad.addColorStop(1, 'rgba(255, 0, 0, 0)');

        ctx.fillStyle = rad;
        ctx.beginPath();
        ctx.arc(targetX, targetY, 22, 0, Math.PI * 2);
        ctx.fill();

        ctx.strokeStyle = '#ffc640';
        ctx.lineWidth = 2;
        ctx.strokeRect(targetX - 18, targetY - 24, 36, 48);

        ctx.fillStyle = '#ffc640';
        ctx.font = '10px "JetBrains Mono", monospace';
        ctx.fillText(`HUMAN ID#${droneNs === 'drone1' ? '102' : '105'} 94.2%`, targetX - 22, targetY - 28);
      }

      animId = requestAnimationFrame(render);
    };

    render();

    return () => {
      cancelAnimationFrame(animId);
    };
  }, [droneNs, cameraMode, hasDetection]);

  return <canvas ref={canvasRef} width={640} height={360} className="synthetic-feed-canvas" />;
}
