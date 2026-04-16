#!/usr/bin/env python3
"""
PET Bottle Robot — Update Server
Run:  python server.py
Then open:  http://<device-ip>:8000/updates
"""

import asyncio
import json
import os
import subprocess
import sys
from pathlib import Path

from fastapi import FastAPI
from fastapi.responses import HTMLResponse, JSONResponse, StreamingResponse

# ── Config ────────────────────────────────────────────────────────────────────
REPO_DIR = Path(__file__).resolve().parent          # git repo root
BRANCH = "main"
SERVER_SCRIPT = Path(__file__).resolve()             # this file (self-restart)

MCU_TARGETS = [
    {
        "name": "PET Bottle Robot (ESP32)",
        "sketch": str(REPO_DIR / "esp32_test" / "esp32_test.ino"),
        "port": "COM3",
        "fqbn": "esp32:esp32:esp32:PartitionScheme=min_spiffs",
        "folder_prefixes": ["esp32_test", "esp32_main"],
    },
]

# ── App ───────────────────────────────────────────────────────────────────────
app = FastAPI(title="PET Bottle Robot Updater")


# ═══════════════════════════════════════════════════════════════════════════════
# 1.  GET /updates  —  Touch-friendly HTML page
# ═══════════════════════════════════════════════════════════════════════════════
@app.get("/updates", response_class=HTMLResponse)
async def updates_page():
    return HTML_PAGE


# ═══════════════════════════════════════════════════════════════════════════════
# 2.  GET /updates/check  —  Compare local HEAD vs origin/main
# ═══════════════════════════════════════════════════════════════════════════════
@app.get("/updates/check")
async def updates_check():
    try:
        # fetch latest from remote
        await _run("git", "fetch", "origin", BRANCH, cwd=REPO_DIR)

        local_sha = (await _run("git", "rev-parse", "HEAD", cwd=REPO_DIR)).strip()
        remote_sha = (
            await _run("git", "rev-parse", f"origin/{BRANCH}", cwd=REPO_DIR)
        ).strip()

        has_update = local_sha != remote_sha

        changelog = ""
        if has_update:
            changelog = (
                await _run(
                    "git", "log", "--oneline", "--no-decorate",
                    f"{local_sha}..{remote_sha}",
                    cwd=REPO_DIR,
                )
            ).strip()

        return JSONResponse(
            {
                "has_update": has_update,
                "local_sha": local_sha[:8],
                "remote_sha": remote_sha[:8],
                "changelog": changelog,
            }
        )
    except Exception as e:
        return JSONResponse({"error": str(e)}, status_code=500)


# ═══════════════════════════════════════════════════════════════════════════════
# 3.  POST /updates/apply  —  Pull, flash MCU if needed, restart server
# ═══════════════════════════════════════════════════════════════════════════════
@app.post("/updates/apply")
async def updates_apply():
    return StreamingResponse(
        _apply_update_stream(), media_type="application/x-ndjson"
    )


async def _apply_update_stream():
    """Generator that yields NDJSON lines as the update progresses."""

    # ── step 1: record current HEAD ──────────────────────────────────────
    yield _ndjson("log", "Fetching latest changes...")
    try:
        old_sha = (await _run("git", "rev-parse", "HEAD", cwd=REPO_DIR)).strip()
        await _run("git", "fetch", "origin", BRANCH, cwd=REPO_DIR)
    except Exception as e:
        yield _ndjson("error", f"Fetch failed: {e}")
        return

    # ── step 2: apply update ─────────────────────────────────────────────
    yield _ndjson("log", "Applying update (git reset --hard)...")
    try:
        await _run("git", "reset", "--hard", f"origin/{BRANCH}", cwd=REPO_DIR)
        new_sha = (await _run("git", "rev-parse", "HEAD", cwd=REPO_DIR)).strip()
        yield _ndjson("log", f"Updated  {old_sha[:8]} -> {new_sha[:8]}")
    except Exception as e:
        yield _ndjson("error", f"Reset failed: {e}")
        return

    # ── step 3: detect what changed ──────────────────────────────────────
    yield _ndjson("log", "Detecting changed files...")
    try:
        diff_output = await _run(
            "git", "diff", "--name-only", old_sha, new_sha, cwd=REPO_DIR
        )
        changed = [f for f in diff_output.strip().splitlines() if f]
        yield _ndjson("log", f"{len(changed)} file(s) changed")
        for f in changed:
            yield _ndjson("file", f)
    except Exception as e:
        yield _ndjson("error", f"Diff failed: {e}")
        changed = []

    # ── step 4: flash microcontrollers if firmware changed ───────────────
    for target in MCU_TARGETS:
        firmware_changed = any(
            any(c.startswith(prefix) for prefix in target["folder_prefixes"])
            for c in changed
        )
        if not firmware_changed:
            yield _ndjson("log", f'{target["name"]}: no firmware changes, skipping flash')
            continue

        yield _ndjson("log", f'{target["name"]}: firmware changed, flashing...')

        # check arduino-cli is available
        arduino_cli = _find_arduino_cli()
        if not arduino_cli:
            yield _ndjson("warn", "arduino-cli not found — skipping flash. Install it to enable OTA firmware updates.")
            continue

        # compile
        yield _ndjson("log", f'{target["name"]}: compiling...')
        try:
            out = await _run(
                arduino_cli, "compile",
                "--fqbn", target["fqbn"],
                target["sketch"],
                cwd=REPO_DIR,
            )
            yield _ndjson("log", f'{target["name"]}: compile OK')
        except Exception as e:
            yield _ndjson("error", f'{target["name"]}: compile failed — {e}')
            continue

        # upload (uses DTR/RTS toggle for ESP32 bootloader)
        yield _ndjson("log", f'{target["name"]}: uploading to {target["port"]}...')
        try:
            # Try manual reset for CP2102 boards (like this project's ESP32)
            await _esp32_manual_reset(target["port"])
            await asyncio.sleep(0.3)

            out = await _run(
                arduino_cli, "upload",
                "--fqbn", target["fqbn"],
                "--port", target["port"],
                target["sketch"],
                cwd=REPO_DIR,
                timeout=120,
            )
            yield _ndjson("log", f'{target["name"]}: flash OK')
        except Exception as e:
            yield _ndjson("warn", f'{target["name"]}: flash failed — {e}. You may need to hold BOOT + press EN on the ESP32 and retry.')

    # ── step 5: check if server code changed & restart ───────────────────
    server_changed = any(
        c.endswith(".py") and not any(
            c.startswith(p) for t in MCU_TARGETS for p in t["folder_prefixes"]
        )
        for c in changed
    )

    if server_changed:
        yield _ndjson("log", "Server code changed — restarting in 3 seconds...")
        yield _ndjson("done", "Update complete! Server is restarting...")
        # give the client time to read the final message
        await asyncio.sleep(3)
        _restart_server()
    else:
        yield _ndjson("done", "Update complete! No server restart needed.")


# ═══════════════════════════════════════════════════════════════════════════════
# Helpers
# ═══════════════════════════════════════════════════════════════════════════════

async def _run(*cmd, cwd=None, timeout=60):
    proc = await asyncio.create_subprocess_exec(
        *[str(c) for c in cmd],
        stdout=asyncio.subprocess.PIPE,
        stderr=asyncio.subprocess.STDOUT,
        cwd=str(cwd) if cwd else None,
    )
    try:
        stdout, _ = await asyncio.wait_for(proc.communicate(), timeout=timeout)
    except asyncio.TimeoutError:
        proc.kill()
        raise RuntimeError(f"Command timed out: {' '.join(str(c) for c in cmd)}")
    if proc.returncode != 0:
        raise RuntimeError(stdout.decode(errors="replace").strip())
    return stdout.decode(errors="replace")


def _ndjson(level: str, message: str) -> str:
    return json.dumps({"level": level, "message": message}) + "\n"


def _find_arduino_cli() -> str | None:
    for name in ("arduino-cli", "arduino-cli.exe"):
        result = subprocess.run(
            ["where" if os.name == "nt" else "which", name],
            capture_output=True, text=True,
        )
        if result.returncode == 0:
            return result.stdout.strip().splitlines()[0]
    return None


async def _esp32_manual_reset(port: str):
    """Toggle DTR/RTS to force ESP32 into bootloader (CP2102 boards)."""
    try:
        import serial
        s = serial.Serial(port, 115200)
        s.dtr = False
        s.rts = True
        await asyncio.sleep(0.1)
        s.dtr = True
        s.rts = False
        await asyncio.sleep(0.05)
        s.dtr = False
        s.close()
    except Exception:
        pass  # pyserial not installed or port busy — let arduino-cli handle it


def _restart_server():
    """Re-exec this process (picks up new code)."""
    os.execv(sys.executable, [sys.executable, str(SERVER_SCRIPT)])


# ═══════════════════════════════════════════════════════════════════════════════
# HTML Page (embedded — no external files)
# ═══════════════════════════════════════════════════════════════════════════════

HTML_PAGE = """<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0, user-scalable=no">
<title>Robot Software Update</title>
<style>
  * { box-sizing: border-box; margin: 0; padding: 0; }
  body {
    font-family: -apple-system, 'Segoe UI', Roboto, sans-serif;
    background: #0f0f0f; color: #e0e0e0;
    display: flex; flex-direction: column; align-items: center;
    justify-content: center; min-height: 100vh; padding: 24px;
    -webkit-tap-highlight-color: transparent;
  }
  .card {
    background: #1a1a2e; border-radius: 20px; padding: 40px 32px;
    max-width: 480px; width: 100%; text-align: center;
    box-shadow: 0 8px 40px rgba(0,0,0,.5);
  }
  h1 { font-size: 1.6rem; margin-bottom: 8px; }
  .subtitle { color: #888; font-size: 0.9rem; margin-bottom: 28px; }

  /* Status icon area */
  .icon { font-size: 64px; margin: 20px 0; }
  .spinner {
    width: 64px; height: 64px; margin: 20px auto;
    border: 6px solid #333; border-top-color: #4fc3f7;
    border-radius: 50%; animation: spin .8s linear infinite;
  }
  @keyframes spin { to { transform: rotate(360deg); } }

  .status-msg { font-size: 1.15rem; margin: 12px 0 4px; font-weight: 600; }
  .sha { font-size: 0.8rem; color: #666; font-family: monospace; margin-bottom: 20px; }
  .changelog {
    text-align: left; background: #111; border-radius: 10px;
    padding: 14px 16px; font-size: 0.82rem; color: #aaa;
    max-height: 160px; overflow-y: auto; margin-bottom: 24px;
    font-family: monospace; white-space: pre-wrap; line-height: 1.5;
  }

  /* Buttons */
  .btn-row { display: flex; gap: 14px; justify-content: center; flex-wrap: wrap; }
  button {
    font-size: 1.1rem; font-weight: 600; border: none; border-radius: 14px;
    padding: 16px 36px; cursor: pointer; transition: transform .1s, opacity .15s;
    min-width: 140px;
  }
  button:active { transform: scale(.96); }
  button:disabled { opacity: .4; cursor: not-allowed; }
  .btn-update { background: #4fc3f7; color: #0a0a0a; }
  .btn-skip   { background: #2a2a3e; color: #aaa; }
  .btn-retry  { background: #ff8a65; color: #0a0a0a; }
  .btn-recheck { background: #2a2a3e; color: #ccc; margin-top: 16px; }

  /* Log area */
  .log {
    text-align: left; background: #111; border-radius: 10px;
    padding: 14px 16px; font-size: 0.8rem; color: #aaa;
    max-height: 260px; overflow-y: auto; margin: 20px 0;
    font-family: monospace; white-space: pre-wrap; line-height: 1.6;
    display: none;
  }
  .log .warn { color: #ffd54f; }
  .log .error { color: #ef5350; }
  .log .done { color: #66bb6a; font-weight: bold; }
  .log .file { color: #4fc3f7; }
</style>
</head>
<body>
<div class="card">
  <h1>Robot Software Update</h1>
  <p class="subtitle">PET Bottle Collector</p>

  <div id="view-loading">
    <div class="spinner"></div>
    <p class="status-msg">Checking for updates...</p>
  </div>

  <div id="view-uptodate" style="display:none">
    <div class="icon">&#10004;</div>
    <p class="status-msg" style="color:#66bb6a">You're up to date!</p>
    <p class="sha" id="sha-current"></p>
    <button class="btn-recheck" onclick="checkForUpdates()">Check Again</button>
  </div>

  <div id="view-available" style="display:none">
    <div class="icon">&#11014;&#65039;</div>
    <p class="status-msg" style="color:#4fc3f7">Update Available</p>
    <p class="sha" id="sha-diff"></p>
    <div class="changelog" id="changelog"></div>
    <div class="btn-row">
      <button class="btn-update" id="btn-apply" onclick="applyUpdate()">Update Now</button>
      <button class="btn-skip" onclick="showView('view-uptodate')">Skip</button>
    </div>
  </div>

  <div id="view-applying" style="display:none">
    <div class="spinner"></div>
    <p class="status-msg">Updating...</p>
    <div class="log" id="log-box"></div>
  </div>

  <div id="view-done" style="display:none">
    <div class="icon">&#10004;</div>
    <p class="status-msg" style="color:#66bb6a" id="done-msg">Update complete!</p>
    <button class="btn-recheck" onclick="location.reload()">Done</button>
  </div>

  <div id="view-error" style="display:none">
    <div class="icon">&#9888;&#65039;</div>
    <p class="status-msg" style="color:#ef5350" id="error-msg">Something went wrong</p>
    <div class="btn-row" style="margin-top:20px">
      <button class="btn-retry" onclick="checkForUpdates()">Try Again</button>
    </div>
  </div>
</div>

<script>
function showView(id) {
  document.querySelectorAll('[id^="view-"]').forEach(v => v.style.display = 'none');
  document.getElementById(id).style.display = '';
}

async function checkForUpdates() {
  showView('view-loading');
  try {
    const r = await fetch('/updates/check');
    const d = await r.json();
    if (d.error) throw new Error(d.error);

    if (d.has_update) {
      document.getElementById('sha-diff').textContent =
        'Installed: ' + d.local_sha + '  \\u2192  Latest: ' + d.remote_sha;
      document.getElementById('changelog').textContent = d.changelog || 'No details available.';
      showView('view-available');
    } else {
      document.getElementById('sha-current').textContent = 'Version: ' + d.local_sha;
      showView('view-uptodate');
    }
  } catch(e) {
    document.getElementById('error-msg').textContent = 'Could not check for updates.';
    showView('view-error');
  }
}

async function applyUpdate() {
  showView('view-applying');
  const logBox = document.getElementById('log-box');
  logBox.style.display = 'block';
  logBox.innerHTML = '';

  try {
    const r = await fetch('/updates/apply', { method: 'POST' });
    const reader = r.body.getReader();
    const decoder = new TextDecoder();
    let buffer = '';

    while (true) {
      const { done, value } = await reader.read();
      if (done) break;
      buffer += decoder.decode(value, { stream: true });

      let lines = buffer.split('\\n');
      buffer = lines.pop();

      for (const line of lines) {
        if (!line.trim()) continue;
        try {
          const entry = JSON.parse(line);
          const span = document.createElement('span');
          span.className = entry.level;
          span.textContent = entry.message + '\\n';
          logBox.appendChild(span);
          logBox.scrollTop = logBox.scrollHeight;

          if (entry.level === 'done') {
            document.getElementById('done-msg').textContent = entry.message;
            setTimeout(() => showView('view-done'), 1200);
          }
        } catch(_) {}
      }
    }

    if (!document.getElementById('view-done').style.display ||
        document.getElementById('view-done').style.display === 'none') {
      document.getElementById('done-msg').textContent = 'Update complete!';
      showView('view-done');
    }
  } catch(e) {
    document.getElementById('error-msg').textContent = 'Update failed. Check the log above.';
    showView('view-error');
  }
}

checkForUpdates();
</script>
</body>
</html>"""


# ═══════════════════════════════════════════════════════════════════════════════
# Run
# ═══════════════════════════════════════════════════════════════════════════════
if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)
