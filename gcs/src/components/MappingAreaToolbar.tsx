import React, { useState } from 'react';
import './MappingAreaToolbar.css';

function formatArea(areaM2: number): string {
  if (areaM2 >= 10000) return `${(areaM2 / 10000).toFixed(2)} ha`;
  return `${Math.round(areaM2).toLocaleString()} m²`;
}

/** Coverage geometry, mirroring nidar_mission_manager/path_planner.py so the
 * operator can see exactly how altitude + camera FOV drive the flight lines
 * (and why a given altitude yields the pattern it does) before pressing
 * Start. Kept in sync with that module's swath_width_m/ground_footprint_m. */
export function coverageStats(altitudeM: number, areaM2: number, droneCount: number,
                              hfovDeg = 60, imgW = 1280, imgH = 960, overlapPct = 20) {
  const aspect = imgW / imgH;
  const vfov = 2 * Math.atan(Math.tan((hfovDeg * Math.PI) / 180 / 2) / aspect);
  const footprintLong = 2 * altitudeM * Math.tan((hfovDeg * Math.PI) / 180 / 2);
  const footprintShort = 2 * altitudeM * Math.tan(vfov / 2);
  const swath = footprintShort; // conservative: short edge, yaw-independent
  const spacing = swath * (1 - overlapPct / 100);
  const zoneArea = droneCount > 0 ? areaM2 / droneCount : areaM2;
  const zoneSide = Math.sqrt(zoneArea);
  const linesPerZone = spacing > 0 ? Math.max(1, Math.ceil(zoneSide / spacing)) : 0;
  return {
    vfovDeg: (vfov * 180) / Math.PI,
    footprintLong, footprintShort, swath, spacing,
    zoneArea, zoneSide, linesPerZone,
    pathPerZone: linesPerZone * zoneSide + Math.max(0, linesPerZone - 1) * spacing,
  };
}

interface MappingAreaToolbarProps {
  isDrawingBoundary: boolean;
  isSettingLaunchSite: boolean;
  isPlacingSurvivor: boolean;
  drawnVertexCount: number;
  hasBoundary: boolean;
  boundaryAreaM2: number;
  altitudeM: number;
  droneCount: number;
  droneNamespaces: string[];
  placingDroneNs: string | null;
  survivorCount: number;
  onToggleDrawBoundary: () => void;
  onToggleSetLaunchSite: () => void;
  onToggleSurvivorPlacement: () => void;
  onRandomizeLaunchSite: () => void;
  onRandomizeDronePositions: () => void;
  onTogglePlaceDrone: (ns: string) => void;
  onResetDronePositions: () => void;
  onClearBoundary: () => void;
  onAddRandomSurvivors: (count: number) => void;
  onClearSurvivors: () => void;
}

export const MappingAreaToolbar: React.FC<MappingAreaToolbarProps> = ({
  isDrawingBoundary,
  isSettingLaunchSite,
  isPlacingSurvivor,
  drawnVertexCount,
  hasBoundary,
  boundaryAreaM2,
  altitudeM,
  droneCount,
  droneNamespaces,
  placingDroneNs,
  survivorCount,
  onToggleDrawBoundary,
  onToggleSetLaunchSite,
  onToggleSurvivorPlacement,
  onRandomizeLaunchSite,
  onRandomizeDronePositions,
  onTogglePlaceDrone,
  onResetDronePositions,
  onClearBoundary,
  onAddRandomSurvivors,
  onClearSurvivors,
}) => {
  const [randomCount, setRandomCount] = useState(5);

  return (
    <div className="mapping-toolbar">
      {hasBoundary && (() => {
        const cs = coverageStats(altitudeM, boundaryAreaM2, droneCount);
        return (
          <div className="mapping-toolbar__group mapping-toolbar__area-panel">
            <span className="mapping-toolbar__area-label">📏 Mapping area</span>
            <span className="mapping-toolbar__area-value">{formatArea(boundaryAreaM2)}</span>
            <span className="mapping-toolbar__area-sep" />
            <span
              className="mapping-toolbar__area-label"
              title={
                `Camera 60° HFOV / ${cs.vfovDeg.toFixed(1)}° VFOV @ ${altitudeM}m\n` +
                `Ground frame ${cs.footprintLong.toFixed(2)}m × ${cs.footprintShort.toFixed(2)}m ` +
                `(${(cs.footprintLong * cs.footprintShort).toFixed(0)} m²)\n` +
                `Swath ${cs.swath.toFixed(2)}m (short edge, yaw-independent), 20% overlap ` +
                `→ line spacing ${cs.spacing.toFixed(2)}m\n` +
                `Each of ${droneCount} zones ≈ ${cs.zoneArea.toFixed(0)} m² ` +
                `(${cs.zoneSide.toFixed(1)}m square) → ${cs.linesPerZone} lines, ` +
                `≈${cs.pathPerZone.toFixed(0)}m of flight`
              }
            >
              🛰️ {cs.linesPerZone} lines/zone · swath {cs.swath.toFixed(1)}m · zone{' '}
              {cs.zoneArea.toFixed(0)} m²
            </span>
          </div>
        );
      })()}
      <div className="mapping-toolbar__group">
        <button
          className={`mapping-toolbar__btn ${isDrawingBoundary ? 'mapping-toolbar__btn--active' : ''}`}
          onClick={onToggleDrawBoundary}
          title="Click points on the map to create a custom mapping boundary"
        >
          {isDrawingBoundary ? '✏️ Drawing Boundary...' : '📐 Draw Mapping Area'}
        </button>

        {isDrawingBoundary && (
          <span className="mapping-toolbar__info">
            {drawnVertexCount < 3
              ? `Click map to add points (${drawnVertexCount}/3 min)`
              : `${drawnVertexCount} points added`}
          </span>
        )}

        {drawnVertexCount > 0 && (
          <button
            className="mapping-toolbar__btn mapping-toolbar__btn--danger"
            onClick={onClearBoundary}
            title="Clear all drawn points"
          >
            🗑️ Clear
          </button>
        )}
      </div>

      <div className="mapping-toolbar__group">
        <button
          className={`mapping-toolbar__btn ${isSettingLaunchSite ? 'mapping-toolbar__btn--active' : ''}`}
          onClick={onToggleSetLaunchSite}
          title="Click anywhere on the map to set the Home Launching Site for the drones"
        >
          {isSettingLaunchSite ? '🎯 Placing Launch Site...' : '🚀 Set Launch Site'}
        </button>

        {hasBoundary && (
          <button
            className="mapping-toolbar__btn mapping-toolbar__btn--secondary"
            onClick={onRandomizeLaunchSite}
            title="Randomly place the Launch Site inside the KML boundary"
          >
            🎲 Randomize Launch Site
          </button>
        )}

        <button
          className="mapping-toolbar__btn mapping-toolbar__btn--secondary"
          onClick={onRandomizeDronePositions}
          title="Randomly place all 4 drones inside the fixed 12ft x 12ft launch/landing box, with safe separation"
        >
          🎲 Randomize Drone Positions
        </button>
      </div>

      <div className="mapping-toolbar__group mapping-toolbar__drone-place">
        <span className="mapping-toolbar__area-label" title="Click a drone, then click inside the 12ft launch box on the map to place it">
          🛩️ Place in box:
        </span>
        {droneNamespaces.map((ns, i) => (
          <button
            key={ns}
            className={`mapping-toolbar__btn mapping-toolbar__btn--drone ${placingDroneNs === ns ? 'mapping-toolbar__btn--active' : ''}`}
            onClick={() => onTogglePlaceDrone(ns)}
            title={`Click here, then click inside the 12ft launch box to place ${ns}`}
          >
            {placingDroneNs === ns ? `🎯 ${ns}...` : `${i}`}
          </button>
        ))}
        <button
          className="mapping-toolbar__btn mapping-toolbar__btn--danger"
          onClick={onResetDronePositions}
          title="Clear all custom drone launch positions (revert to default formation)"
        >
          ↺ Reset
        </button>
      </div>

      {hasBoundary && (
        <div className="mapping-toolbar__group">
          <button
            className={`mapping-toolbar__btn mapping-toolbar__btn--survivor ${isPlacingSurvivor ? 'mapping-toolbar__btn--active-survivor' : ''}`}
            onClick={onToggleSurvivorPlacement}
            title="Click inside the boundary to place a survivor dummy"
          >
            {isPlacingSurvivor ? '🎯 Placing Survivor...' : '🧍 Place Survivor'}
          </button>

          <input
            type="number"
            min={1}
            max={50}
            className="mapping-toolbar__count-input"
            value={randomCount}
            onChange={(e) => setRandomCount(Math.max(1, Number(e.target.value) || 1))}
            title="Number of random survivors to add"
          />
          <button
            className="mapping-toolbar__btn mapping-toolbar__btn--secondary"
            onClick={() => onAddRandomSurvivors(randomCount)}
            title="Randomly scatter survivors inside the boundary"
          >
            🎲 Add {randomCount} Random
          </button>

          {survivorCount > 0 && (
            <button
              className="mapping-toolbar__btn mapping-toolbar__btn--danger"
              onClick={onClearSurvivors}
              title="Remove all placed survivors"
            >
              🗑️ Clear Survivors ({survivorCount})
            </button>
          )}
        </div>
      )}
    </div>
  );
};
