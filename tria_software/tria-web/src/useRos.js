import { useCallback, useEffect, useRef, useState } from "react";
import ROSLIB from "roslib";

// ここを自分の環境に合わせて変更
export const CONFIG = {
  mapTopic: "/map", // nav_msgs/msg/OccupancyGrid
  poseTopic: "/odom", // PoseWithCovarianceStamped / Odometry / PoseStamped のどれでもOK
  commandTopic: "/web_ui/command", // std_msgs/msg/String (JSON) を Publish
  mapFrame: "map",
};

const yawFromQuat = (q) =>
  Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));

export function useRos(url) {
  const [status, setStatus] = useState("connecting"); // connecting | connected | error
  const [map, setMap] = useState(null);
  const [pose, setPose] = useState(null);
  const rosRef = useRef(null);
  const cmdTopicRef = useRef(null);

  useEffect(() => {
    let closed = false;
    let retryTimer = null;
    let ros = null;
    let topics = [];

    const connect = () => {
      setStatus("connecting");
      ros = new ROSLIB.Ros({ url });
      rosRef.current = ros;

      ros.on("connection", () => {
        setStatus("connected");

        // 地図（大きいので png 圧縮）
        const mapTopic = new ROSLIB.Topic({
          ros,
          name: CONFIG.mapTopic,
          messageType: "nav_msgs/msg/OccupancyGrid",
          compression: "png",
        });
        mapTopic.subscribe((m) => setMap(m));

        // 自己位置
        const poseTopic = new ROSLIB.Topic({
          ros,
          name: CONFIG.poseTopic,
          throttle_rate: 100, // ms
        });
        poseTopic.subscribe((m) => {
          // PoseWithCovarianceStamped / Odometry は m.pose.pose、PoseStamped は m.pose
          const p = m.pose?.pose ?? m.pose;
          if (!p) return;
          setPose({
            x: p.position.x,
            y: p.position.y,
            yaw: yawFromQuat(p.orientation),
          });
        });

        cmdTopicRef.current = new ROSLIB.Topic({
          ros,
          name: CONFIG.commandTopic,
          messageType: "std_msgs/msg/String",
        });
        cmdTopicRef.current.advertise();

        topics = [mapTopic, poseTopic, cmdTopicRef.current];
      });

      ros.on("error", () => setStatus("error"));
      ros.on("close", () => {
        setStatus("error");
        topics = [];
        cmdTopicRef.current = null;
        if (!closed) retryTimer = setTimeout(connect, 2000); // 自動再接続
      });
    };

    connect();
    return () => {
      closed = true;
      clearTimeout(retryTimer);
      topics.forEach((t) => t.unsubscribe?.());
      ros?.close();
    };
  }, [url]);

  // mode: "collect" | "place"
  const sendTarget = useCallback((mode, x, y) => {
    const topic = cmdTopicRef.current;
    if (!topic) return false;
    const payload = {
      mode,
      x,
      y,
      frame_id: CONFIG.mapFrame,
      stamp: Date.now() / 1000,
    };
    topic.publish(new ROSLIB.Message({ data: JSON.stringify(payload) }));
    return payload;
  }, []);

  return { status, map, pose, sendTarget };
}
