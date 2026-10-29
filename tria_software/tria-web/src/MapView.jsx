import { useEffect, useMemo, useRef, useState } from "react";

// OccupancyGrid -> オフスクリーン canvas（行を上下反転: ROSは左下原点）
function renderGrid(map) {
  const { width, height } = map.info;
  const off = document.createElement("canvas");
  off.width = width;
  off.height = height;
  const ctx = off.getContext("2d");
  const img = ctx.createImageData(width, height);
  const d = map.data;
  for (let j = 0; j < height; j++) {
    const row = (height - 1 - j) * width;
    for (let i = 0; i < width; i++) {
      const v = d[j * width + i];
      const c = v < 0 ? 205 : v >= 65 ? 30 : 250 - Math.round(v * 1.2); // 未知/占有/空き
      const k = (row + i) * 4;
      img.data[k] = img.data[k + 1] = img.data[k + 2] = c;
      img.data[k + 3] = 255;
    }
  }
  ctx.putImageData(img, 0, 0);
  return off;
}

export default function MapView({ map, pose, target, mode, onTap }) {
  const wrapRef = useRef(null);
  const canvasRef = useRef(null);
  const [box, setBox] = useState({ w: 0, h: 0 });
  const grid = useMemo(() => (map ? renderGrid(map) : null), [map]);

  // 表示エリアのサイズを監視
  useEffect(() => {
    const el = wrapRef.current;
    if (!el) return;
    const ro = new ResizeObserver(([e]) =>
      setBox({ w: e.contentRect.width, h: e.contentRect.height })
    );
    ro.observe(el);
    return () => ro.disconnect();
  }, []);

  // 縦横比を保ったまま、表示エリアいっぱいに拡大/縮小
  const fit = map && box.w ? Math.min(box.w / map.info.width, box.h / map.info.height) : 0;
  const dispW = map ? map.info.width * fit : 0;
  const dispH = map ? map.info.height * fit : 0;

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas || !grid || !dispW) return;
    const { width, height, resolution } = map.info;
    const { x: ox, y: oy } = map.info.origin.position;

    // 実表示解像度で描画（拡大してもマーカーがぼやけない）
    const dpr = window.devicePixelRatio || 1;
    canvas.width = Math.round(dispW * dpr);
    canvas.height = Math.round(dispH * dpr);
    const k = canvas.width / width; // 1セルあたりのcanvas px
    const ctx = canvas.getContext("2d");
    ctx.imageSmoothingEnabled = false;
    ctx.drawImage(grid, 0, 0, canvas.width, canvas.height);

    const toPx = (x, y) => [((x - ox) / resolution) * k, (height - (y - oy) / resolution) * k];

    // タップ先
    if (target) {
      const [px, py] = toPx(target.x, target.y);
      ctx.strokeStyle = ctx.fillStyle = target.mode === "collect" ? "#0b6bcb" : "#7a3fc2";
      ctx.lineWidth = 3 * dpr;
      ctx.beginPath();
      ctx.arc(px, py, 11 * dpr, 0, Math.PI * 2);
      ctx.stroke();
      ctx.beginPath();
      ctx.arc(px, py, 3.5 * dpr, 0, Math.PI * 2);
      ctx.fill();
    }

    // 自己位置（三角形で向きも表示）
    if (pose) {
      const [px, py] = toPx(pose.x, pose.y);
      const r = 12 * dpr;
      ctx.save();
      ctx.translate(px, py);
      ctx.rotate(-pose.yaw);
      ctx.beginPath();
      ctx.moveTo(r * 1.4, 0);
      ctx.lineTo(-r, r * 0.9);
      ctx.lineTo(-r * 0.5, 0);
      ctx.lineTo(-r, -r * 0.9);
      ctx.closePath();
      ctx.fillStyle = "#e0342b";
      ctx.strokeStyle = "#fff";
      ctx.lineWidth = 2 * dpr;
      ctx.fill();
      ctx.stroke();
      ctx.restore();
    }
  }, [grid, map, pose, target, dispW, dispH]);

  const handleClick = (e) => {
    if (!map || !mode) return;
    const rect = canvasRef.current.getBoundingClientRect();
    const { width, height, resolution } = map.info;
    const { x: ox, y: oy } = map.info.origin.position;
    const cx = ((e.clientX - rect.left) / rect.width) * width; // セル座標
    const cy = ((e.clientY - rect.top) / rect.height) * height;
    onTap({ x: ox + cx * resolution, y: oy + (height - cy) * resolution });
  };

  return (
    <div className="map-fit" ref={wrapRef}>
      {map ? (
        <canvas
          ref={canvasRef}
          className={"map-canvas" + (mode ? " armed" : "")}
          style={{ width: dispW, height: dispH }}
          onClick={handleClick}
        />
      ) : (
        <div className="map-empty">
          地図を待っています。rosbridge に接続し、/map が配信されているか確認してください。
        </div>
      )}
    </div>
  );
}