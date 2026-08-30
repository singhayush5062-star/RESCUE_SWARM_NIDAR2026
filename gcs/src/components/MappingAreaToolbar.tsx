import React from 'react';
import './MappingAreaToolbar.css';

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
  onGenerateRandomBoundary?: () => void;
  onRandomizeLaunchSite: () => void;
  onRandomizeDronePositions: () => void;
  onScatterDronesNearby: () => void;
  hasLaunchSite: boolean;
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
  droneNamespaces,
  placingDroneNs,
  survivorCount,
  onToggleDrawBoundary,
  onToggleSetLaunchSite,
  onToggleSurvivorPlacement,
  onGenerateRandomBoundary,
  onRandomizeLaunchSite,
  onRandomizeDronePositions,
  onScatterDronesNearby,
  hasLaunchSite,
  onTogglePlaceDrone,
  onResetDronePositions,
  onClearBoundary,
  onAddRandomSurvivors,
  onClearSurvivors,
}) => {
  const formatArea = (m2: number) => {
    if (m2 > 10000) return `${(m2 / 10000).toFixed(2)} ha`;
    return `${Math.round(m2)} m²`;
  };

  return (
    <div className="mapping-toolbar-glass">
      <div className="obsidian-card-header" style={{ marginBottom: 0 }}>
        <div style={{ display: 'flex', alignItems: 'center', gap: 6 }}>
          <span className="icon" style={{ color: 'var(--primary-bright)' }}>
            polyline
          </span>
          <span>MISSION PLANNING SUITE</span>
        </div>
        <span className="obsidian-badge badge-primary">PRE-FLIGHT SETUP</span>
      </div>

      {/* 1. MAPPING AREA BOUNDARY TOOLS */}
      <div className="toolbar-section-box">
        <div className="toolbar-section-header">
          <span>1. ARENA BOUNDARY SELECTION</span>
          <span className="telemetry-val" style={{ color: 'var(--primary-bright)' }}>
            {hasBoundary ? formatArea(boundaryAreaM2) : `${drawnVertexCount} PTS`}
          </span>
        </div>

        <div className="toolbar-btn-row">
          <button
            className={`glow-btn glow-btn-cyan ${isDrawingBoundary ? 'active' : ''}`}
            onClick={onToggleDrawBoundary}
          >
            <span className="icon">{isDrawingBoundary ? 'check_circle' : 'draw'}</span>
            {isDrawingBoundary ? `DONE (${drawnVertexCount} PTS)` : 'DRAW ARENA (MAP CLICK)'}
          </button>
        </div>

        <div className="toolbar-btn-row">
          {onGenerateRandomBoundary && (
            <button className="glow-btn glow-btn-cyan" onClick={onGenerateRandomBoundary}>
              <span className="icon">shuffle</span>
              GENERATE RANDOM ARENA
            </button>
          )}

          {(hasBoundary || drawnVertexCount > 0) && (
            <button className="glow-btn" onClick={onClearBoundary} title="Clear Boundary">
              <span className="icon">delete</span>
              CLEAR
            </button>
          )}
        </div>
      </div>

      {/* 2. LAUNCH & LANDING STATION (12ft x 12ft) */}
      <div className="toolbar-section-box">
        <div className="toolbar-section-header">
          <span>2. LAUNCH & LANDING STATION</span>
          <span className="obsidian-badge badge-success">FIXED 12 FT x 12 FT BOX</span>
        </div>

        <div className="launch-box-info-pill">
          <span className="icon" style={{ fontSize: 16 }}>
            crop_square
          </span>
          <span>All drones launch & land inside 12 ft x 12 ft (3.65m) landing zone</span>
        </div>

        <div className="toolbar-btn-row">
          <button
            className={`glow-btn glow-btn-green ${isSettingLaunchSite ? 'active' : ''}`}
            onClick={onToggleSetLaunchSite}
          >
            <span className="icon">my_location</span>
            {isSettingLaunchSite ? 'CLICK MAP FOR LAUNCH SITE' : 'SET LAUNCH SITE (CLICK)'}
          </button>

          <button className="glow-btn glow-btn-green" onClick={onRandomizeLaunchSite}>
            <span className="icon">casino</span>
            RANDOM LAUNCH SITE
          </button>
        </div>
      </div>

      {/* 3. DRONE FORMATION PLACEMENT (inside 12ft Box) */}
      <div className="toolbar-section-box">
        <div className="toolbar-section-header">
          <span>3. DRONE LAUNCH FORMATION</span>
          <span className="telemetry-val" style={{ color: 'var(--text-muted)' }}>
            4 DRONES
          </span>
        </div>

        <div className="toolbar-btn-row">
          {/* Scattering inside the box needs a box. Before a launch site
              exists the swarm still has to go somewhere sensible, so the
              nearby scatter below works off the drones' own GPS instead --
              this button used to be silently dead in that state (its handler
              began `if (!center) return;`). */}
          <button
            className="glow-btn glow-btn-green"
            onClick={onRandomizeDronePositions}
            disabled={!hasLaunchSite}
            title={hasLaunchSite
              ? 'Scatter the four drones at random points inside the 12ft launch box'
              : 'Set a launch site first — this scatters drones inside the 12ft box'}
          >
            <span className="icon">grid_view</span>
            RANDOM DRONE FORMATION
          </button>
          <button className="glow-btn" onClick={onResetDronePositions}>
            RESET
          </button>
        </div>

        <div className="toolbar-btn-row">
          {/* Exactly complementary to the button above: the backend only
              enforces the 12ft box once a mission with a `home` is loaded
              (mission_executor `_launch_box_center`). Before that, this
              scatters ~6 m around the swarm's own GPS; after it, a 6 m
              scatter would be rejected as outside the box, so the in-box
              formation button is the one that applies. */}
          <button
            className="glow-btn"
            onClick={onScatterDronesNearby}
            disabled={hasLaunchSite}
            title={hasLaunchSite
              ? 'A launch site is set — drones must stay inside the 12ft box; use RANDOM DRONE FORMATION'
              : "Scatter the drones at random points within ~6 m of the swarm's own GPS position"}
          >
            <span className="icon">scatter_plot</span>
            SCATTER NEAR GPS
          </button>
        </div>

        <div style={{ fontSize: 10, color: 'var(--text-muted)', fontFamily: 'var(--font-mono)' }}>
          PLACE INDIVIDUAL DRONE IN LAUNCH BOX:
        </div>
        <div className="toolbar-btn-row">
          {droneNamespaces.map((ns) => {
            const isPlacing = placingDroneNs === ns;
            return (
              <button
                key={ns}
                className={`glow-btn ${isPlacing ? 'active' : ''}`}
                style={{ padding: '4px 6px', fontSize: 10 }}
                onClick={() => onTogglePlaceDrone(ns)}
              >
                {ns.toUpperCase().replace('DRONE', 'D-')}
              </button>
            );
          })}
        </div>
      </div>

      {/* 4. SURVIVOR DUMMY SCATTER & PLACEMENT */}
      <div className="toolbar-section-box">
        <div className="toolbar-section-header">
          <span>4. SURVIVOR DUMMIES</span>
          <span className="telemetry-val" style={{ color: 'var(--status-warning)' }}>
            {survivorCount} PLACED
          </span>
        </div>

        <div className="toolbar-btn-row">
          <button
            className={`glow-btn glow-btn-amber ${isPlacingSurvivor ? 'active' : ''}`}
            onClick={onToggleSurvivorPlacement}
          >
            <span className="icon">person_add</span>
            {isPlacingSurvivor ? 'CLICK MAP TO DROP' : 'CLICK PLACE SURVIVOR'}
          </button>

          <button className="glow-btn glow-btn-amber" onClick={() => onAddRandomSurvivors(3)}>
            <span className="icon">groups</span>
            ADD 3 SURVIVORS
          </button>

          {survivorCount > 0 && (
            <button className="glow-btn" onClick={onClearSurvivors} title="Clear Survivors">
              CLEAR
            </button>
          )}
        </div>
      </div>
    </div>
  );
};
