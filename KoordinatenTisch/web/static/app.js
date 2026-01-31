async function postJson(url) {
  const res = await fetch(url, { method: "POST" });
  return await res.json();
}

async function fetchStatus() {
  try {
    const res = await fetch("/camera/status");
    const data = await res.json();

    const started = data.started ? "RUNNING" : (data.opened ? "OPEN" : "OFF");
    document.getElementById("v_started").textContent = started;

    document.getElementById("v_model").textContent = data.model ?? "—";
    document.getElementById("v_serial").textContent = data.serial ?? "—";

    const w = data.width ?? data.meta_width ?? data.w ?? null;
    const h = data.height ?? data.meta_height ?? data.h ?? null;
    document.getElementById("v_res").textContent = (w && h) ? `${w} x ${h}` : "—";

    document.getElementById("v_pf").textContent = data.pixel_format ?? "—";
    document.getElementById("v_decode").textContent = data.decode_label ?? "—";

    document.getElementById("v_streamfps").textContent =
      (data.stream_fps != null) ? Number(data.stream_fps).toFixed(1) : "—";

    document.getElementById("v_camfps").textContent =
      (data.camera_fps != null) ? Number(data.camera_fps).toFixed(1) : "—";

    document.getElementById("v_exp").textContent =
      (data.exposure_us != null) ? Math.round(Number(data.exposure_us)).toString() : "—";

    document.getElementById("v_gain").textContent =
      (data.gain_db != null) ? Number(data.gain_db).toFixed(2) : "—";

    const binH = data.bin_h ?? 1;
    const binV = data.bin_v ?? 1;
    document.getElementById("v_bin").textContent = `${binH} x ${binV}`;

    const decH = data.dec_h ?? 1;
    const decV = data.dec_v ?? 1;
    document.getElementById("v_dec").textContent = `${decH} x ${decV}`;
  } catch (e) {
  }
}

function setupButtons() {
  const btnStart = document.getElementById("btnStart");
  const btnStop = document.getElementById("btnStop");

  btnStart.addEventListener("click", async () => {
    await postJson("/camera/start");
    await fetchStatus();
    // Stream-IMG neu laden (falls caching)
    const img = document.getElementById("camImg");
    img.src = "/camera/stream?ts=" + Date.now();
  });

  btnStop.addEventListener("click", async () => {
    await postJson("/camera/stop");
    await fetchStatus();
  });
}

setupButtons();
fetchStatus();
setInterval(fetchStatus, 500);
