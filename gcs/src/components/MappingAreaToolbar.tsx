import React, { useState } from 'react';
import './MappingAreaToolbar.css';

interface MappingAreaToolbarProps {
  isDrawingBoundary: boolean;
  isSettingLaunchSite: boolean;
  isPlacingSurvivor: boolean;
  drawnVertexCount: number;
  hasBoundary: boolean;
  survivorCount: number;
  onToggleDrawBoundary: () => void;
  onToggleSetLaunchSite: () => void;
  onToggleSurvivorPlacement: () => void;
  onRandomizeLaunchSite: () => void;
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
  survivorCount,
  onToggleDrawBoundary,
  onToggleSetLaunchSite,
  onToggleSurvivorPlacement,
  onRandomizeLaunchSite,
  onClearBoundary,
  onAddRandomSurvivors,
  onClearSurvivors,
}) => {
  const [randomCount, setRandomCount] = useState(5);

  return (
    <div className="mapping-toolbar">
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
