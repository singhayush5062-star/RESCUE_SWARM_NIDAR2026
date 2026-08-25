# Walkthrough: Interactive Mapping Area Selection & Custom Launch Site Placement

We have implemented an interactive GCS map interface allowing operators to draw mapping boundaries point-by-point, select or randomize drone **Home Launch Sites** on the map, and dynamically execute lawnmower mapping missions using `map.kml` or custom-drawn boundaries.

---

## 1. Key Accomplishments

### A. Dynamic Interactive Mapping Area Selection
- **Toolbar Component ([MappingAreaToolbar.tsx](file:///home/ayush/NIDAR/gcs/src/components/MappingAreaToolbar.tsx)):** Added an interactive control bar above the map with toggles for boundary drawing, launch site selection, and polygon clearing.
- **Point-and-Click Boundary Vertices:** Clicking on the Leaflet map while `📐 Draw Mapping Area` is active places yellow vertex markers that automatically connect into an outer boundary polygon.

### B. Customizable Launch Site & Drone Placement
- **Interactive Launch Site Selector ([launchSiteManager.ts](file:///home/ayush/NIDAR/gcs/src/mission/launchSiteManager.ts)):**
  - Clicking the map under `🚀 Set Launch Site` mode places the **Home Launching Site** (`mission.home`) at that coordinate.
  - Clicking `🎲 Randomize Launch Site` samples a valid coordinate inside the KML boundary polygon using ray-casting point-in-polygon math.
- **Drone Grid Formation Preview:** Automatically calculates 4-drone launch positions (`drone0`..`drone3`) around the selected Launching Site and previews them as color-coded markers on the map.

### C. KML Coordinate Alignment & Mission Execution
- **Fixed `map.kml` Simulation Offset:** Updated [mission_file_executor.py](file:///home/ayush/NIDAR/project_gazebo/mission_file_executor.py) to check `mission['home']`. When a user loads `map.kml` or sets a launch site, the `zone_splitter` and `path_planner` use `mission.home` as the reference origin for lawnmower path generation, fixing out-of-bounds mapping errors.

---

## 2. Changes Made

| File Path | Description of Changes |
| :--- | :--- |
| [gcs/src/mission/launchSiteManager.ts](file:///home/ayush/NIDAR/gcs/src/mission/launchSiteManager.ts) | **[NEW]** Spatial GIS utility module providing `isPointInPolygon`, `generateRandomLaunchSiteInKml`, and multi-drone grid formation math. |
| [gcs/src/components/MappingAreaToolbar.tsx](file:///home/ayush/NIDAR/gcs/src/components/MappingAreaToolbar.tsx) | **[NEW]** Interactive toolbar providing boundary drawing toggles, launch site selector, and reset buttons. |
| [gcs/src/components/MappingAreaToolbar.css](file:///home/ayush/NIDAR/gcs/src/components/MappingAreaToolbar.css) | **[NEW]** Modern dark-theme glassmorphism styling for the interactive toolbar. |
| [gcs/src/components/MapView.tsx](file:///home/ayush/NIDAR/gcs/src/components/MapView.tsx) | **[MODIFY]** Integrated `useMapEvents` for click-to-draw vertex placement, high-resolution tile layers (up to zoom level 22), Launch Site marker, and drone placement previews. |
| [gcs/src/App.tsx](file:///home/ayush/NIDAR/gcs/src/App.tsx) | **[MODIFY]** Wired interactive state for boundary drawing, launch site placement, toolbar toggles, and mission creation. |
| [project_gazebo/mission_file_executor.py](file:///home/ayush/NIDAR/project_gazebo/mission_file_executor.py) | **[MODIFY]** Updated `_run_boundary_coverage_mission` to use `mission['home']` as reference origin for zone splitting and lawnmower coverage path planning. |

---

## 3. Verification & Validation Results

### Automated Tests & Compilation
- **TypeScript Compilation:** Verified zero errors across GCS app:
  ```bash
  cd /home/ayush/NIDAR/gcs && npx tsc --noEmit
  # Output: Completed successfully with 0 errors
  ```

### Functional Verification
1. **Interactive Boundary Drawing:**
   - Activated `📐 Draw Mapping Area`. Clicked vertices on Leaflet map. Verified yellow vertex pins and outer polygon boundaries rendered cleanly.
2. **Launch Site Placement & Randomization:**
   - Activated `🚀 Set Launch Site` and clicked on map. Verified `🚀 Home Launching Site` green pin updated immediately along with drone formation preview markers.
   - Clicked `🎲 Randomize Launch Site`. Verified launch site was picked strictly inside the boundary polygon.
3. **KML Mission Dispatch:**
   - Loaded `project_gazebo/missions/map.kml`. Verified centroid launch site calculation and dynamic mission generation.
