import React, { useState, useRef, useEffect } from 'react';
import type { LogEntry } from '../types/gcs';
import './LogsConsolePanel.css';

interface LogsConsolePanelProps {
  logs: LogEntry[];
  onClearLogs?: () => void;
  droneNamespaces: string[];
}

export const LogsConsolePanel: React.FC<LogsConsolePanelProps> = ({
  logs,
  onClearLogs,
  droneNamespaces,
}) => {
  const [levelFilter, setLevelFilter] = useState<'ALL' | 'INFO' | 'WARN' | 'ERROR' | 'CMD'>('ALL');
  const [sourceFilter, setSourceFilter] = useState<string>('ALL');
  const [searchQuery, setSearchQuery] = useState('');
  const [autoScroll, setAutoScroll] = useState(true);

  const outputRef = useRef<HTMLDivElement>(null);

  useEffect(() => {
    if (autoScroll && outputRef.current) {
      outputRef.current.scrollTop = outputRef.current.scrollHeight;
    }
  }, [logs, autoScroll]);

  // Sources actually present in the buffer, rather than a fixed list. The
  // backend's names are ROS node names -- 'mission_executor',
  // 'detection_drone0', 'drone2.FollowPathBehavior' -- so the old hardcoded
  // options ('SwarmManager', bare 'drone0') matched nothing at all once real
  // /rosout lines started arriving.
  const presentSources = Array.from(new Set(logs.map((l) => l.source))).sort();

  const filteredLogs = logs.filter((log) => {
    if (levelFilter !== 'ALL' && log.level !== levelFilter) return false;
    if (sourceFilter !== 'ALL') {
      // Substring, not equality: picking "drone2" must catch every node that
      // speaks for that drone -- drone2.FollowPathBehavior,
      // drone2.TakeoffBehavior, detection_drone2 -- which is the question an
      // operator is actually asking when they filter by a drone.
      if (!log.source.includes(sourceFilter)) return false;
    }
    if (searchQuery && !log.message.toLowerCase().includes(searchQuery.toLowerCase())) return false;
    return true;
  });

  return (
    <div className="logs-console-container">
      <div className="logs-toolbar">
        <div className="logs-filters">
          <span style={{ color: 'var(--text-muted)', fontSize: 11, fontWeight: 700 }}>LEVEL:</span>
          {(['ALL', 'INFO', 'WARN', 'ERROR', 'CMD'] as const).map((lvl) => (
            <button
              key={lvl}
              className={`log-filter-btn ${levelFilter === lvl ? 'active' : ''}`}
              onClick={() => setLevelFilter(lvl)}
            >
              {lvl}
            </button>
          ))}
        </div>

        <div className="logs-filters">
          <span style={{ color: 'var(--text-muted)', fontSize: 11, fontWeight: 700 }}>SOURCE:</span>
          <select
            className="log-search-input"
            style={{ width: 'auto' }}
            value={sourceFilter}
            onChange={(e) => setSourceFilter(e.target.value)}
          >
            <option value="ALL">ALL SOURCES</option>
            {droneNamespaces.map((ns) => (
              <option key={ns} value={ns}>
                {ns.toUpperCase()} (all nodes)
              </option>
            ))}
            {presentSources
              .filter((src) => !droneNamespaces.some((ns) => src.includes(ns)))
              .map((src) => (
                <option key={src} value={src}>
                  {src}
                </option>
              ))}
          </select>

          <input
            type="text"
            className="log-search-input"
            placeholder="Search logs..."
            value={searchQuery}
            onChange={(e) => setSearchQuery(e.target.value)}
          />

          <button
            className={`log-filter-btn ${autoScroll ? 'active' : ''}`}
            onClick={() => setAutoScroll(!autoScroll)}
            title="Toggle Auto Scroll"
          >
            <span className="icon" style={{ fontSize: 12 }}>
              south
            </span>
            AUTO-SCROLL
          </button>

          {onClearLogs && (
            <button className="log-filter-btn" onClick={onClearLogs} title="Clear Terminal Logs">
              CLEAR
            </button>
          )}
        </div>
      </div>

      <div className="logs-output" ref={outputRef}>
        {filteredLogs.length === 0 ? (
          <div style={{ color: 'var(--text-subtle)', fontStyle: 'italic', padding: 20, textAlign: 'center' }}>
            No log entries match the selected filter.
          </div>
        ) : (
          filteredLogs.map((log) => (
            <div key={log.id} className="log-line">
              <span className="log-timestamp">[{log.timestamp}]</span>
              <span className={`log-level ${log.level}`}>{log.level}</span>
              <span className="log-source">[{log.source}]</span>
              <span className="log-msg">{log.message}</span>
            </div>
          ))
        )}
      </div>
    </div>
  );
};
