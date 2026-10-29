# ROS2 自律走行コンソール

```bash
npm install
npm run dev
```

ロボット側:

```bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml   # ws://<robot>:9090
```

トピック名は `src/useRos.js` の `CONFIG` で変更。

## Publish されるメッセージ

`/web_ui/command` (`std_msgs/msg/String`) に JSON:

```json
{"mode":"collect","x":1.23,"y":-0.45,"frame_id":"map","stamp":1730000000.123}
```

`mode` は `"collect"`（回収）または `"place"`（設置）。
