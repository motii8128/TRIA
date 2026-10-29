import { useState } from "react";
import MapView from "./MapView.jsx";
import { useRos } from "./useRos.js";

const MODES = [
  { id: "collect", label: "回収モード", hint: "地図をタップすると、その場所を回収地点として送信します" },
  { id: "place", label: "設置モード", hint: "地図をタップすると、その場所を設置地点として送信します" },
];

const STATUS_TEXT = {
  connecting: "接続中…",
  connected: "接続済み",
  error: "未接続（再接続を試行中）",
};

export default function App() {
  const [url, setUrl] = useState(`ws://${location.hostname}:9090`);
  const [draftUrl, setDraftUrl] = useState(url);
  const [mode, setMode] = useState(null); // null = 表示のみ
  const [target, setTarget] = useState(null);
  const [log, setLog] = useState([]);
  const { status, map, pose, sendTarget } = useRos(url);

  const handleTap = ({ x, y }) => {
    const sent = sendTarget(mode, x, y);
    if (!sent) {
      setLog((l) => [{ ok: false, text: "送信できません（未接続）" }, ...l].slice(0, 8));
      return;
    }
    setTarget({ mode, x, y });
    const label = MODES.find((m) => m.id === mode).label;
    setLog((l) => [{ ok: true, text: `${label}  x=${x.toFixed(2)}  y=${y.toFixed(2)}` }, ...l].slice(0, 8));
  };

  const current = MODES.find((m) => m.id === mode);

  return (
    <div className="app">
      <header className="bar">
        <h1>TRIA-WEB</h1>
        <span className={"dot " + status} />
        <span className="status">{STATUS_TEXT[status]}</span>
        <form
          className="url"
          onSubmit={(e) => {
            e.preventDefault();
            setUrl(draftUrl);
          }}
        >
          <input value={draftUrl} onChange={(e) => setDraftUrl(e.target.value)} aria-label="rosbridge URL" />
          <button type="submit">接続</button>
        </form>
      </header>

      <main className="main">
        <section className="map-pane">
          <MapView map={map} pose={pose} target={target} mode={mode} onTap={handleTap} />
        </section>

        <aside className="side">
          <div className="group">
            <h2>操作モード</h2>
            <div className="modes">
              {MODES.map((m) => (
                <button
                  key={m.id}
                  className={`mode ${m.id}` + (mode === m.id ? " on" : "")}
                  onClick={() => setMode(mode === m.id ? null : m.id)}
                  aria-pressed={mode === m.id}
                >
                  {m.label}
                </button>
              ))}
            </div>
            <p className="hint">{current ? current.hint : "モードを選ぶと、地図のタップで指示を送れます（現在は表示のみ）"}</p>
          </div>

          <div className="group">
            <h2>自己位置</h2>
            {pose ? (
              <dl className="pose">
                <dt>x</dt><dd>{pose.x.toFixed(2)} m</dd>
                <dt>y</dt><dd>{pose.y.toFixed(2)} m</dd>
                <dt>向き</dt><dd>{((pose.yaw * 180) / Math.PI).toFixed(1)}°</dd>
              </dl>
            ) : (
              <p className="hint">自己位置を受信していません</p>
            )}
          </div>

          <div className="group">
            <h2>送信履歴</h2>
            {log.length === 0 ? (
              <p className="hint">まだ送信していません</p>
            ) : (
              <ul className="log">
                {log.map((l, i) => (
                  <li key={i} className={l.ok ? "" : "ng"}>{l.text}</li>
                ))}
              </ul>
            )}
          </div>
        </aside>
      </main>
    </div>
  );
}
