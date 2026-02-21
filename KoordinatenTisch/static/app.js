let activeIndex = -1;
let lastVisitedKey = '';
let lastDrawKey = '';
let lastWorkAreaKey = '';
let lastStartPos = null;
let lastVisited = [];
let tableCanvas = null;
let lastMpos = null;

let camFeatureCache = [];
let grblSettingsCache = {};

let workAreaX = 200.0;
let workAreaY = 150.0;
let focusThreshold = 0.0;

let uiRunning = false;

function setUiRunning(running) {
  const r = !!running;
  if (r === uiRunning) return;
  uiRunning = r;

  const btnStop = document.getElementById('btnStop');

  // Main controls
  const btnSetStart = document.getElementById('btnSetStart');
  const btnRun = document.getElementById('btnRun');
  const demoToggle = document.getElementById('demoToggle');
  if (btnSetStart) btnSetStart.disabled = r;
  if (btnRun) btnRun.disabled = r;
  if (demoToggle) demoToggle.disabled = r;

  // Jog buttons
  const jogBtns = document.querySelectorAll('[data-jog]');
  jogBtns.forEach((b) => {
    try { b.disabled = r; } catch (_) {}
  });

  // Settings button + modal controls that can change config
  const btnOpenSettings = document.getElementById('btnOpenSettings');
  if (btnOpenSettings) btnOpenSettings.disabled = r;
  const btnApplyWorkArea = document.getElementById('btnApplyWorkArea');
  if (btnApplyWorkArea) btnApplyWorkArea.disabled = r;
  const focusThresholdEl = document.getElementById('focusThreshold');
  if (focusThresholdEl) focusThresholdEl.disabled = r;
  const workAreaXEl = document.getElementById('workAreaX');
  const workAreaYEl = document.getElementById('workAreaY');
  if (workAreaXEl) workAreaXEl.disabled = r;
  if (workAreaYEl) workAreaYEl.disabled = r;
  const btnReloadCam = document.getElementById('btnReloadCam');
  if (btnReloadCam) btnReloadCam.disabled = r;
  const btnReloadGrbl = document.getElementById('btnReloadGrbl');
  if (btnReloadGrbl) btnReloadGrbl.disabled = r;

  // Visited list: prevent accidental moves
  const listEl = document.getElementById('posList');
  if (listEl) listEl.classList.toggle('disabled', r);

  // Stop must remain possible
  if (btnStop) btnStop.disabled = false;

  if (r) {
    // Ensure no jog continues if a run begins.
    postJson('/api/jog/stop', {}).catch(() => {});
  }
}

function debounce(fn, delayMs) {
  let t = null;
  return (...args) => {
    if (t) clearTimeout(t);
    t = setTimeout(() => fn(...args), delayMs);
  };
}

async function postJson(url, body) {
  const res = await fetch(url, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(body || {})
  });
  return await res.json();
}

function escapeHtml(s) {
  return String(s)
    .replaceAll('&', '&amp;')
    .replaceAll('<', '&lt;')
    .replaceAll('>', '&gt;')
    .replaceAll('"', '&quot;')
    .replaceAll("'", '&#39;');
}

function fmtVal(v) {
  if (v === null || v === undefined) return '';
  if (typeof v === 'number') return String(v);
  if (typeof v === 'boolean') return v ? 'true' : 'false';
  return String(v);
}

function visitedKeyFast(visited) {
  if (!Array.isArray(visited)) return '';
  const n = visited.length;
  if (n === 0) return '0';
  const last = visited[n - 1];
  if (!Array.isArray(last) || last.length < 2) return String(n);
  return n + ':' + String(last[0]) + ',' + String(last[1]);
}

function posKey(p, decimals) {
  if (!Array.isArray(p) || p.length < 2) return '';
  const d = (typeof decimals === 'number' && decimals >= 0) ? decimals : 2;
  const x = (typeof p[0] === 'number') ? p[0].toFixed(d) : String(p[0]);
  const y = (typeof p[1] === 'number') ? p[1].toFixed(d) : String(p[1]);
  const z = (p.length >= 3 && typeof p[2] === 'number') ? p[2].toFixed(d) : '';
  return x + ',' + y + (z ? (',' + z) : '');
}

async function loadCamFeatures() {
  const wrap = document.getElementById('camFeatures');
  if (!wrap) return;
  wrap.innerHTML = '<div class="small">Lade…</div>';

  try {
    const res = await fetch('/api/camera/features');
    const data = await res.json();
    if (!data.ok) throw new Error(data.error || 'failed');
    camFeatureCache = data.features || [];
    renderCamFeatures(camFeatureCache);
  } catch (e) {
    wrap.innerHTML = `<div class="small">Fehler: ${escapeHtml(e.message || e)}</div>`;
  }
}

function renderCamFeatures(features) {
  const wrap = document.getElementById('camFeatures');
  if (!wrap) return;

  if (!features || features.length === 0) {
    wrap.innerHTML = '<div class="small">Keine setzbaren Features gefunden.</div>';
    return;
  }

  const debouncedSet = debounce(async (name, value, statusEl) => {
    statusEl.textContent = '…';
    const r = await postJson('/api/camera/features/' + encodeURIComponent(name), { value });
    if (!r.ok) {
      statusEl.textContent = 'Fehler';
      statusEl.className = 'statusTag err';
      return;
    }
    statusEl.textContent = 'OK';
    statusEl.className = 'statusTag ok';
  }, 150);

  const list = document.createElement('div');
  list.className = 'featureList';

  for (const f of features) {
    const name = f.name;
    const iface = f.interface || '';
    const unit = f.unit || '';
    const desc = f.description || '';
    const min = f.min;
    const max = f.max;
    const inc = f.inc;
    const enumValues = f.enum_values;

    const row = document.createElement('div');
    row.className = 'featureRow';

    const top = document.createElement('div');
    top.className = 'featureTop';

    const left = document.createElement('div');
    left.className = 'featureName';
    left.innerHTML = `${escapeHtml(name)} <span class="pill">${escapeHtml(iface || '—')}</span>`;

    const meta = document.createElement('div');
    meta.className = 'featureMeta';
    if (unit) meta.innerHTML += `<span class="pill">${escapeHtml(unit)}</span>`;
    if (typeof min === 'number' && typeof max === 'number') meta.innerHTML += `<span class="pill">${min}..${max}</span>`;
    if (typeof inc === 'number') meta.innerHTML += `<span class="pill">step ${inc}</span>`;

    top.appendChild(left);
    top.appendChild(meta);

    const ctl = document.createElement('div');
    ctl.className = 'featureCtl';

    const statusEl = document.createElement('span');
    statusEl.className = 'statusTag';
    statusEl.textContent = '';

    if (iface === 'IBoolean') {
      const chk = document.createElement('input');
      chk.type = 'checkbox';
      chk.checked = !!f.value;
      chk.addEventListener('change', () => {
        debouncedSet(name, chk.checked, statusEl);
      });
      ctl.appendChild(chk);
      const val = document.createElement('span');
      val.className = 'small';
      val.textContent = chk.checked ? 'Ein' : 'Aus';
      chk.addEventListener('change', () => { val.textContent = chk.checked ? 'Ein' : 'Aus'; });
      ctl.appendChild(val);
    } else if (iface === 'IEnumeration' && Array.isArray(enumValues) && enumValues.length > 0) {
      const sel = document.createElement('select');
      for (const ev of enumValues) {
        const opt = document.createElement('option');
        opt.value = ev;
        opt.textContent = ev;
        sel.appendChild(opt);
      }
      try { sel.value = String(f.value); } catch (_) {}
      sel.addEventListener('change', () => {
        debouncedSet(name, sel.value, statusEl);
      });
      ctl.appendChild(sel);
    } else if (iface === 'IFloat' || iface === 'IInteger') {
      const num = document.createElement('input');
      num.type = 'number';
      if (typeof min === 'number') num.min = String(min);
      if (typeof max === 'number') num.max = String(max);
      if (typeof inc === 'number' && inc > 0) num.step = String(inc);
      num.value = fmtVal(f.value);
      num.addEventListener('input', () => {
        const v = num.value;
        if (v === '') return;
        debouncedSet(name, iface === 'IInteger' ? parseInt(v, 10) : parseFloat(v), statusEl);
      });
      ctl.appendChild(num);

      if (typeof min === 'number' && typeof max === 'number' && isFinite(min) && isFinite(max) && max > min) {
        const rng = document.createElement('input');
        rng.type = 'range';
        rng.min = String(min);
        rng.max = String(max);
        rng.step = (typeof inc === 'number' && inc > 0) ? String(inc) : '1';
        const cur = (typeof f.value === 'number') ? f.value : parseFloat(num.value || String(min));
        rng.value = String(isFinite(cur) ? cur : min);
        rng.addEventListener('input', () => {
          num.value = rng.value;
          const v = rng.value;
          debouncedSet(name, iface === 'IInteger' ? parseInt(v, 10) : parseFloat(v), statusEl);
        });
        ctl.appendChild(rng);
      }
    } else if (iface === 'ICommand') {
      const btn = document.createElement('button');
      btn.className = 'btn mini';
      btn.textContent = 'Execute';
      btn.addEventListener('click', async () => {
        statusEl.textContent = '…';
        const r = await postJson('/api/camera/features/' + encodeURIComponent(name), { execute: true });
        statusEl.textContent = r.ok ? 'OK' : 'Fehler';
      });
      ctl.appendChild(btn);
    } else {
      const txt = document.createElement('input');
      txt.type = 'text';
      txt.value = fmtVal(f.value);
      const btn = document.createElement('button');
      btn.className = 'btn mini';
      btn.textContent = 'Set';
      btn.addEventListener('click', () => {
        debouncedSet(name, txt.value, statusEl);
      });
      ctl.appendChild(txt);
      ctl.appendChild(btn);
    }

    ctl.appendChild(statusEl);

    const hint = document.createElement('div');
    hint.className = 'featureHint';
    hint.textContent = desc || '';

    row.appendChild(top);
    row.appendChild(ctl);
    if (desc) row.appendChild(hint);
    list.appendChild(row);
  }

  wrap.innerHTML = '';
  wrap.appendChild(list);
}

async function loadGrblSettings() {
  const wrap = document.getElementById('grblSettings');
  if (!wrap) return;
  wrap.innerHTML = '<div class="small">Lade…</div>';
  try {
    const res = await fetch('/api/grbl/settings');
    const data = await res.json();
    if (!data.ok) throw new Error(data.error || 'failed');
    grblSettingsCache = data.settings || {};
    renderGrblSettings(grblSettingsCache);
  } catch (e) {
    wrap.innerHTML = `<div class="small">Fehler: ${escapeHtml(e.message || e)}</div>`;
  }
}

function renderGrblSettings(settings) {
  const wrap = document.getElementById('grblSettings');
  if (!wrap) return;
  const keys = Object.keys(settings || {}).sort((a,b) => {
    const na = parseInt(a.replace('$',''),10);
    const nb = parseInt(b.replace('$',''),10);
    if (isFinite(na) && isFinite(nb)) return na - nb;
    return a.localeCompare(b);
  });

  if (keys.length === 0) {
    wrap.innerHTML = '<div class="small">Keine $$ Settings verfügbar (oder Demo/keine Verbindung).</div>';
    return;
  }

  const list = document.createElement('div');
  list.className = 'kvList';

  for (const key of keys) {
    const meta = settings[key] || {};
    const row = document.createElement('div');
    row.className = 'kvRow';

    const top = document.createElement('div');
    top.className = 'kvTop';
    const k = document.createElement('div');
    k.className = 'kvKey';
    k.textContent = key;
    top.appendChild(k);
    row.appendChild(top);

    const ctl = document.createElement('div');
    ctl.className = 'kvCtl';
    const input = document.createElement('input');
    input.type = 'text';
    input.value = meta.value ?? '';
    const btn = document.createElement('button');
    btn.className = 'btn mini';
    btn.textContent = 'Set';
    const status = document.createElement('span');
    status.className = 'statusTag';
    status.textContent = '';
    btn.addEventListener('click', async () => {
      status.textContent = '…';
      const r = await postJson('/api/grbl/settings/' + encodeURIComponent(key), { value: input.value });
      status.textContent = r.ok ? 'OK' : 'Fehler';
      if (r.ok) loadGrblSettings();
    });
    ctl.appendChild(input);
    ctl.appendChild(btn);
    ctl.appendChild(status);
    row.appendChild(ctl);

    if (meta.description) {
      const d = document.createElement('div');
      d.className = 'kvDesc';
      d.textContent = meta.description;
      row.appendChild(d);
    }

    list.appendChild(row);
  }

  wrap.innerHTML = '';
  wrap.appendChild(list);
}

function fmtTime(seconds) {
  if (!seconds && seconds !== 0) return '—';
  const s = Math.max(0, seconds);
  if (s < 60) return s.toFixed(1) + ' s';
  const m = Math.floor(s / 60);
  const r = s - m * 60;
  return m + ' min ' + r.toFixed(0) + ' s';
}

function resizeTableCanvas() {
  const canvas = document.getElementById('tableCanvas');
  if (!canvas) return;

  const parent = canvas.parentElement;
  if (!parent) return;

  const wrapW = Math.floor(parent.clientWidth || 520);
  const wrapH = Math.floor(parent.clientHeight || 0);
  let widthPx = Math.max(240, wrapW);
  const ratio = (typeof workAreaX === 'number' && isFinite(workAreaX) && workAreaX > 0)
    ? (workAreaY / workAreaX)
    : (360 / 520);
  let desiredHeight = Math.max(200, Math.floor(widthPx * (isFinite(ratio) && ratio > 0 ? ratio : 0.69)));

  // If the computed height doesn't fit, clamp to available height and recompute width.
  if (wrapH > 220 && desiredHeight > (wrapH - 4)) {
    desiredHeight = Math.max(200, wrapH - 4);
    if (isFinite(ratio) && ratio > 0) {
      widthPx = Math.max(240, Math.floor(desiredHeight / ratio));
    }
  }

  if (canvas.width !== widthPx || canvas.height !== desiredHeight) {
    canvas.width = widthPx;
    canvas.height = desiredHeight;
  }
}

function drawTable(canvas, startPos, visited, currentMpos) {
  const ctx = canvas.getContext('2d');
  const w = canvas.width;
  const h = canvas.height;
  ctx.clearRect(0, 0, w, h);

  const points = [];
  if (startPos) points.push({ x: startPos[0], y: startPos[1], kind: 'start', idx: -1 });
  for (let i = 0; i < (visited || []).length; i++) {
    points.push({ x: visited[i][0], y: visited[i][1], kind: 'pt', idx: i });
  }
  if (Array.isArray(currentMpos) && currentMpos.length >= 2) {
    points.push({ x: currentMpos[0], y: currentMpos[1], kind: 'cur', idx: -2 });
  }

  // bounds: fixed machine work area (0..X, 0..Y)
  const minX = 0.0;
  const minY = 0.0;
  const maxX = (typeof workAreaX === 'number' && isFinite(workAreaX) && workAreaX > 0) ? workAreaX : 1.0;
  const maxY = (typeof workAreaY === 'number' && isFinite(workAreaY) && workAreaY > 0) ? workAreaY : 1.0;

  const pad = 18;
  const spanX = Math.max(1e-6, maxX - minX);
  const spanY = Math.max(1e-6, maxY - minY);
  const scale = Math.min((w - 2 * pad) / spanX, (h - 2 * pad) / spanY);

  function toCanvas(p) {
    const cx = pad + (p.x - minX) * scale;
    // invert Y for typical table view
    const cy = h - pad - (p.y - minY) * scale;
    return { cx, cy };
  }

  // work area box
  ctx.strokeStyle = '#ddd';
  const tl = toCanvas({ x: minX, y: maxY });
  const br = toCanvas({ x: maxX, y: minY });
  ctx.strokeRect(tl.cx, tl.cy, br.cx - tl.cx, br.cy - tl.cy);

  if (points.length === 0) {
    ctx.fillStyle = '#666';
    ctx.fillText('Keine Punkte', 10, 20);
    return;
  }

  // start
  if (startPos) {
    const p = { x: startPos[0], y: startPos[1] };
    const { cx, cy } = toCanvas(p);
    ctx.fillStyle = '#111';
    ctx.beginPath();
    ctx.arc(cx, cy, 5, 0, Math.PI * 2);
    ctx.fill();
  }

  // path: start -> visited...
  if (startPos && (visited || []).length > 0) {
    ctx.strokeStyle = '#444';
    ctx.lineWidth = 2;
    ctx.beginPath();
    const s = toCanvas({ x: startPos[0], y: startPos[1] });
    ctx.moveTo(s.cx, s.cy);
    for (let i = 0; i < visited.length; i++) {
      const p = toCanvas({ x: visited[i][0], y: visited[i][1] });
      ctx.lineTo(p.cx, p.cy);
    }
    ctx.stroke();
    ctx.lineWidth = 1;
  }

  // visited
  for (let i = 0; i < (visited || []).length; i++) {
    const p = { x: visited[i][0], y: visited[i][1] };
    const { cx, cy } = toCanvas(p);
    const isActive = i === activeIndex;

    ctx.fillStyle = isActive ? '#111' : '#444';
    ctx.beginPath();
    ctx.arc(cx, cy, isActive ? 6 : 4, 0, Math.PI * 2);
    ctx.fill();

    if (isActive) {
      ctx.strokeStyle = '#111';
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.arc(cx, cy, 10, 0, Math.PI * 2);
      ctx.stroke();
      ctx.lineWidth = 1;
    }
  }

  // current position marker
  if (Array.isArray(currentMpos) && currentMpos.length >= 2) {
    const p = toCanvas({ x: currentMpos[0], y: currentMpos[1] });
    ctx.strokeStyle = '#111';
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(p.cx - 8, p.cy);
    ctx.lineTo(p.cx + 8, p.cy);
    ctx.moveTo(p.cx, p.cy - 8);
    ctx.lineTo(p.cx, p.cy + 8);
    ctx.stroke();
    ctx.lineWidth = 1;
  }
}

function setList(listEl, visited) {
  listEl.innerHTML = '';
  for (let i = 0; i < visited.length; i++) {
    const li = document.createElement('li');
    li.textContent = `${i + 1}: ${visited[i][0].toFixed(2)}, ${visited[i][1].toFixed(2)}`;
    li.dataset.index = String(i);

    li.addEventListener('mouseenter', () => {
      activeIndex = i;
      updateActive(listEl);
      if (tableCanvas) drawTable(tableCanvas, lastStartPos, lastVisited, lastMpos);
    });
    li.addEventListener('mouseleave', () => {
      activeIndex = -1;
      updateActive(listEl);
      if (tableCanvas) drawTable(tableCanvas, lastStartPos, lastVisited, lastMpos);
    });
    li.addEventListener('click', async () => {
      if (uiRunning) return;
      await postJson('/api/move_to', { x: visited[i][0], y: visited[i][1] });
    });

    listEl.appendChild(li);
  }
  updateActive(listEl);
}

function updateActive(listEl) {
  const items = listEl.querySelectorAll('li');
  items.forEach((li) => {
    li.classList.toggle('active', Number(li.dataset.index) === activeIndex);
  });
}

function setupJogButtons() {
  const btns = document.querySelectorAll('[data-jog]');
  for (const btn of btns) {
    const dir = btn.getAttribute('data-jog');

    const start = async (e) => {
      e.preventDefault();
      await postJson('/api/jog/start', { direction: dir });
    };
    const stop = async (e) => {
      e.preventDefault();
      await postJson('/api/jog/stop', {});
    };

    btn.addEventListener('mousedown', start);
    btn.addEventListener('mouseup', stop);
    btn.addEventListener('mouseleave', stop);

    btn.addEventListener('touchstart', start, { passive: false });
    btn.addEventListener('touchend', stop, { passive: false });
    btn.addEventListener('touchcancel', stop, { passive: false });
  }
}

async function poll() {
  const res = await fetch('/api/status');
  const data = await res.json();

  const statusLine = document.getElementById('statusLine');
  const mStart = document.getElementById('mStart');
  const mTime = document.getElementById('mTime');
  const mCount = document.getElementById('mCount');
  const mAcc = document.getElementById('mAcc');
  const mMsg = document.getElementById('mMsg');
  const listEl = document.getElementById('posList');
  const canvas = document.getElementById('tableCanvas');
  tableCanvas = canvas;

  if (data.work_area) {
    if (typeof data.work_area.x_mm === 'number') workAreaX = data.work_area.x_mm;
    if (typeof data.work_area.y_mm === 'number') workAreaY = data.work_area.y_mm;
  }
  const waKey = String(workAreaX) + 'x' + String(workAreaY);
  if (waKey !== lastWorkAreaKey) {
    lastWorkAreaKey = waKey;
    resizeTableCanvas();
  }

  const grbl = data.grbl || {};
  const cam = data.camera || {};
  const camText = cam.connected ? 'Kamera: verbunden' : 'Kamera: nicht verbunden';

  if (grbl.error) {
    statusLine.textContent = `Status: GRBL Fehler (${grbl.error})  |  ${camText}`;
  } else {
    const mpos = grbl.mpos;
    const wpos = grbl.wpos;
    if (Array.isArray(mpos) && mpos.length >= 3) {
      statusLine.textContent = `Status: ${grbl.state || '—'}  |  MPos X${mpos[0].toFixed(2)} Y${mpos[1].toFixed(2)} Z${mpos[2].toFixed(2)}  |  ${camText}`;
    } else if (Array.isArray(wpos) && wpos.length >= 3) {
      statusLine.textContent = `Status: ${grbl.state || '—'}  |  WPos X${wpos[0].toFixed(2)} Y${wpos[1].toFixed(2)} Z${wpos[2].toFixed(2)}  |  ${camText}`;
    } else {
      statusLine.textContent = `Status: ${grbl.state || '—'}  |  ${camText}`;
    }
  }

  const metrics = data.metrics || {};
  setUiRunning(!!metrics.running);

  const sp = data.start_pos;
  if (Array.isArray(sp) && sp.length >= 3) {
    mStart.textContent = `X${sp[0].toFixed(2)} Y${sp[1].toFixed(2)} Z${sp[2].toFixed(2)}`;
  } else if (Array.isArray(sp) && sp.length >= 2) {
    mStart.textContent = `X${sp[0].toFixed(2)} Y${sp[1].toFixed(2)}`;
  } else {
    mStart.textContent = '—';
  }

  mTime.textContent = fmtTime(metrics.elapsed_s);
  mCount.textContent = String(metrics.points_visited ?? '—');
  const acc = metrics.mean_error_mm;
  mAcc.textContent = (typeof acc === 'number') ? (acc.toFixed(3) + ' mm') : '—';
  mMsg.textContent = metrics.last_message || '—';

  const visited = data.visited || [];
  lastVisited = visited;
  lastStartPos = data.start_pos;
  lastMpos = (data.grbl && (data.grbl.mpos || data.grbl.wpos)) ? (data.grbl.mpos || data.grbl.wpos) : null;
  const visitedKey = visitedKeyFast(visited);
  if (visitedKey !== lastVisitedKey) {
    lastVisitedKey = visitedKey;
    setList(listEl, visited);
  } else {
    updateActive(listEl);
  }

  // Only redraw the canvas if something relevant changed.
  const drawKey = [
    visitedKey,
    posKey(lastStartPos, 2),
    posKey(lastMpos, 1),
    String(activeIndex),
    String(canvas ? canvas.width : 0),
    String(canvas ? canvas.height : 0),
    waKey,
  ].join('|');

  if (drawKey !== lastDrawKey) {
    lastDrawKey = drawKey;
    drawTable(canvas, lastStartPos, visited, lastMpos);
  }
}

function setupActions() {
  const demoToggle = document.getElementById('demoToggle');
  const workAreaXEl = document.getElementById('workAreaX');
  const workAreaYEl = document.getElementById('workAreaY');
  const btnApplyWorkArea = document.getElementById('btnApplyWorkArea');
  const focusThresholdEl = document.getElementById('focusThreshold');

  function applyWorkAreaFromInputs() {
    const x = workAreaXEl ? parseFloat(workAreaXEl.value) : NaN;
    const y = workAreaYEl ? parseFloat(workAreaYEl.value) : NaN;
    if (!(isFinite(x) && x > 0 && isFinite(y) && y > 0)) return;

    postJson('/api/settings', { work_area: { x_mm: x, y_mm: y } })
      .then((r) => {
        if (r && r.ok && r.work_area) {
          workAreaX = Number(r.work_area.x_mm) || workAreaX;
          workAreaY = Number(r.work_area.y_mm) || workAreaY;
          if (workAreaXEl) workAreaXEl.value = String(workAreaX);
          if (workAreaYEl) workAreaYEl.value = String(workAreaY);
          resizeTableCanvas();
          if (tableCanvas) drawTable(tableCanvas, lastStartPos, lastVisited, lastMpos);
        }
      })
      .catch(() => {});
  }

  function applyFocusThresholdFromInput() {
    if (!focusThresholdEl) return;
    const v = parseFloat(focusThresholdEl.value);
    const t = (isFinite(v) && v >= 0) ? v : 0;
    postJson('/api/settings', { focus_threshold: t })
      .then((r) => {
        if (r && r.ok) {
          if (typeof r.focus_threshold === 'number') focusThreshold = r.focus_threshold;
          focusThresholdEl.value = String(focusThreshold);
        }
      })
      .catch(() => {});
  }

  const debouncedApplyWorkArea = debounce(applyWorkAreaFromInputs, 250);
  const debouncedApplyFocusThreshold = debounce(applyFocusThresholdFromInput, 250);

  // initialize toggle from backend
  fetch('/api/settings').then(r => r.json()).then(s => {
    if (demoToggle) demoToggle.checked = !!s.demo_mode;
    const wa = s.work_area || {};
    if (typeof wa.x_mm === 'number') workAreaX = wa.x_mm;
    if (typeof wa.y_mm === 'number') workAreaY = wa.y_mm;
    if (typeof s.focus_threshold === 'number') focusThreshold = s.focus_threshold;
    if (workAreaXEl) workAreaXEl.value = String(workAreaX);
    if (workAreaYEl) workAreaYEl.value = String(workAreaY);
    if (focusThresholdEl) focusThresholdEl.value = String(focusThreshold);
    resizeTableCanvas();
  }).catch(() => {});

  if (demoToggle) {
    demoToggle.addEventListener('change', async () => {
      const enabled = demoToggle.checked;
      await postJson('/api/settings', { demo_mode: enabled });

      // force reconnect video stream
      const video = document.getElementById('video');
      if (video) video.src = '/video_feed?ts=' + Date.now();

      // reset caches
      lastVisitedKey = '';
      activeIndex = -1;
      await poll();
    });
  }

  if (btnApplyWorkArea) btnApplyWorkArea.addEventListener('click', applyWorkAreaFromInputs);
  if (workAreaXEl) workAreaXEl.addEventListener('input', debouncedApplyWorkArea);
  if (workAreaYEl) workAreaYEl.addEventListener('input', debouncedApplyWorkArea);
  if (focusThresholdEl) focusThresholdEl.addEventListener('input', debouncedApplyFocusThreshold);

  document.getElementById('btnSetStart').addEventListener('click', async () => {
    await postJson('/api/set_start', {});
  });
  document.getElementById('btnRun').addEventListener('click', async () => {
    await postJson('/api/run_program', {});
  });
  document.getElementById('btnStop').addEventListener('click', async () => {
    await postJson('/api/stop_program', {});
    await postJson('/api/jog/stop', {});
  });

  const btnReloadCam = document.getElementById('btnReloadCam');
  if (btnReloadCam) btnReloadCam.addEventListener('click', loadCamFeatures);
  const btnReloadGrbl = document.getElementById('btnReloadGrbl');
  if (btnReloadGrbl) btnReloadGrbl.addEventListener('click', loadGrblSettings);

  // Settings modal (open/close + draggable)
  const settingsModal = document.getElementById('settingsModal');
  const btnOpenSettings = document.getElementById('btnOpenSettings');
  const btnCloseSettings = document.getElementById('btnCloseSettings');
  const settingsModalHeader = document.getElementById('settingsModalHeader');

  function openSettingsModal() {
    if (!settingsModal) return;
    settingsModal.classList.add('open');
    settingsModal.setAttribute('aria-hidden', 'false');
  }

  function closeSettingsModal() {
    if (!settingsModal) return;
    settingsModal.classList.remove('open');
    settingsModal.setAttribute('aria-hidden', 'true');
  }

  if (btnOpenSettings) btnOpenSettings.addEventListener('click', openSettingsModal);
  if (btnCloseSettings) btnCloseSettings.addEventListener('click', closeSettingsModal);
  window.addEventListener('keydown', (e) => {
    if (e.key === 'Escape') closeSettingsModal();
  });

  if (settingsModal && settingsModalHeader) {
    let dragActive = false;
    let startX = 0;
    let startY = 0;
    let originLeft = 0;
    let originTop = 0;

    function clamp(v, lo, hi) {
      return Math.max(lo, Math.min(hi, v));
    }

    settingsModalHeader.addEventListener('pointerdown', (e) => {
      // don't start drag when clicking close button
      const t = e.target;
      if (t && t.closest && t.closest('button')) return;
      if (e.button !== undefined && e.button !== 0) return;

      dragActive = true;
      startX = e.clientX;
      startY = e.clientY;

      const rect = settingsModal.getBoundingClientRect();
      originLeft = rect.left;
      originTop = rect.top;

      settingsModal.style.right = 'auto';
      settingsModal.style.bottom = 'auto';
      settingsModal.style.left = originLeft + 'px';
      settingsModal.style.top = originTop + 'px';

      settingsModalHeader.setPointerCapture(e.pointerId);
      e.preventDefault();
    });

    settingsModalHeader.addEventListener('pointermove', (e) => {
      if (!dragActive) return;

      const dx = e.clientX - startX;
      const dy = e.clientY - startY;

      const rect = settingsModal.getBoundingClientRect();
      const w = rect.width;
      const h = rect.height;

      const nextLeft = clamp(originLeft + dx, 6, window.innerWidth - w - 6);
      const nextTop = clamp(originTop + dy, 6, window.innerHeight - h - 6);

      settingsModal.style.left = nextLeft + 'px';
      settingsModal.style.top = nextTop + 'px';
    });

    settingsModalHeader.addEventListener('pointerup', (e) => {
      dragActive = false;
      try { settingsModalHeader.releasePointerCapture(e.pointerId); } catch (_) {}
    });
    settingsModalHeader.addEventListener('pointercancel', (e) => {
      dragActive = false;
      try { settingsModalHeader.releasePointerCapture(e.pointerId); } catch (_) {}
    });
  }
}

window.addEventListener('load', () => {
  setupJogButtons();
  setupActions();
  resizeTableCanvas();
  window.addEventListener('resize', () => {
    resizeTableCanvas();
    if (tableCanvas) drawTable(tableCanvas, lastStartPos, lastVisited, lastMpos);
  });
  poll();
  loadCamFeatures();
  loadGrblSettings();
  setInterval(poll, 500);
});
