import { useEffect, useRef, useState } from 'react';
import * as ROSLIB from 'roslib';
import { rosConnection } from './RosConnection';
import type { LogEntry } from '../types/gcs';

/** One line as nidar_gcs_bridge serialises it onto /gcs/log. */
interface WireLog {
  level: string;
  source: string;
  message: string;
  stamp: number;
}

/** Most recent lines kept in memory. A mission produces a few thousand at
 * 3.5/s; keeping them all would grow without bound over a long session, and
 * nobody scrolls back past a few hundred. */
const MAX_ENTRIES = 500;

/** How often accumulated lines are pushed into React state.
 *
 * Deliberately NOT per message. Calling setState on every log line re-renders
 * the whole console at message rate, which is the single most likely way to
 * make this panel feel laggy -- the same mistake the video feed already cost
 * this project once. Lines land in a ref immediately and are flushed on this
 * interval, so arrival is never delayed by more than this. */
const FLUSH_MS = 250;

const LEVELS: Record<string, LogEntry['level']> = {
  DEBUG: 'INFO', INFO: 'INFO', WARN: 'WARN', ERROR: 'ERROR', FATAL: 'ERROR',
};

/**
 * Streams the backend's own log output into the GCS console.
 *
 * Source is `/gcs/log`, which nidar_gcs_bridge relays from `/rosout` after
 * filtering to operator-relevant nodes, flooring the severity, and rate
 * capping. Subscribing to `/rosout` directly from the browser would work but
 * would ship every node in the graph (120 publishers in a 4-drone run),
 * most of it transform-listener noise.
 *
 * No `throttle_rate`: the backend already caps the rate, and rosbridge
 * throttling adds latency on top of the source rate rather than replacing it
 * -- see the note in useDetections.ts.
 */
export function useMissionLog() {
  const [entries, setEntries] = useState<LogEntry[]>([]);
  const pending = useRef<LogEntry[]>([]);
  const seq = useRef(0);

  useEffect(() => {
    let topic: ROSLIB.Topic<{ data: string }> | null = null;

    const unsubscribe = rosConnection.onStateChange((state) => {
      if (state !== 'connected') return;
      topic = new ROSLIB.Topic<{ data: string }>({
        ros: rosConnection.getRos(),
        name: '/gcs/log',
        messageType: 'std_msgs/msg/String',
        queue_length: 0,
      });
      topic.subscribe((msg) => {
        try {
          const w: WireLog = JSON.parse(msg.data);
          pending.current.push({
            id: `rosout-${seq.current++}`,
            timestamp: new Date(w.stamp * 1000).toLocaleTimeString(),
            sortKey: w.stamp * 1000,
            level: LEVELS[w.level] ?? 'INFO',
            source: w.source,
            message: w.message,
          });
        } catch {
          // a malformed line must not take the console down
        }
      });
    });

    const flush = setInterval(() => {
      if (pending.current.length === 0) return;
      const batch = pending.current;
      pending.current = [];
      setEntries((prev) => [...prev, ...batch].slice(-MAX_ENTRIES));
    }, FLUSH_MS);

    return () => {
      unsubscribe();
      clearInterval(flush);
      topic?.unsubscribe();
    };
  }, []);

  const counts = entries.reduce(
    (acc, e) => {
      if (e.level === 'WARN') acc.warn += 1;
      if (e.level === 'ERROR') acc.error += 1;
      return acc;
    },
    { warn: 0, error: 0 },
  );

  return { entries, counts, clear: () => setEntries([]) };
}
