// server.js — HTTPS + WSS static server for web_vr with proxy for MJPEG stream
// Place in ~/tele_vr_robot/web_vr/server.js and run: node server.js
// Requires: cert/key at ./cert/cert.pem and ./cert/key.pem
// Proxies: https://<host>:8443/camera/stream  ->  http://127.0.0.1:8082/stream

const fs = require('fs');
const path = require('path');
const express = require('express');
const https = require('https');
const http = require('http');
const { WebSocketServer } = require('ws');

const PORT = process.env.PORT || 8443;
const HOST = process.env.HOST || '0.0.0.0';
const ROOT = path.join(__dirname);
const CERT_DIR = path.join(__dirname, 'cert');
const KEY_PATH = path.join(CERT_DIR, 'key.pem');
const CERT_PATH = path.join(CERT_DIR, 'cert.pem');

// --- check certs ---
if (!fs.existsSync(KEY_PATH) || !fs.existsSync(CERT_PATH)) {
  console.error('SSL cert or key missing in ./cert (key.pem, cert.pem required). Exiting.');
  process.exit(1);
}

// --- express app ---
const app = express();

// small security headers (safe for local dev)
app.use((req, res, next) => {
  res.setHeader('X-Content-Type-Options', 'nosniff');
  res.setHeader('X-Frame-Options', 'DENY');
  next();
});

// serve static files from repo root (index_vr.html, assets/, node_modules/)
app.use(express.static(ROOT, {
  index: false,
  extensions: ['html', 'htm']
}));

// fallback route to index_vr.html
app.get(['/', '/index_vr.html', '/index.html'], (req, res) => {
  res.sendFile(path.join(ROOT, 'index_vr.html'));
});

// health
app.get('/_health', (req, res) => res.json({ ok: true }));

// ----- MJPEG proxy to image_streamer -----
const UPSTREAM_HOST = '127.0.0.1';
const UPSTREAM_PORT = 8082;
const UPSTREAM_PATHS = ['/stream', '/camera/stream'];

UPSTREAM_PATHS.forEach(p => {
  app.get(p, (req, res) => {
    // Proxy request to http://127.0.0.1:8082{p}
    const options = {
      hostname: UPSTREAM_HOST,
      port: UPSTREAM_PORT,
      path: p,
      method: 'GET',
      headers: {
        'Accept': 'multipart/x-mixed-replace, */*'
      },
      timeout: 5000
    };

    const upstreamReq = http.request(options, (upRes) => {
      // forward headers and status
      res.writeHead(upRes.statusCode || 200, upRes.headers);
      upRes.pipe(res, { end: true });
    });

    upstreamReq.on('error', (err) => {
      console.error('Upstream proxy error:', err.message || err);
      if (!res.headersSent) res.status(502).send('Bad Gateway');
    });

    upstreamReq.on('timeout', () => {
      upstreamReq.destroy();
    });

    upstreamReq.end();
  });
});

// create https server
const server = https.createServer({
  key: fs.readFileSync(KEY_PATH),
  cert: fs.readFileSync(CERT_PATH)
}, app);

// --- WebSocket server ---
const wss = new WebSocketServer({ server });
let clientIdSeq = 0;
const clients = new Map();

// throttled logger to avoid terminal spam
let lastLogTime = 0;
function throttledLog(msg) {
  const now = Date.now();
  if ((now - lastLogTime) > 800) {
    console.log(msg);
    lastLogTime = now;
  }
}

wss.on('connection', (ws, req) => {
  const id = ++clientIdSeq;
  clients.set(id, ws);
  console.log(`WS client connected: ${id} (${req.socket.remoteAddress})`);
  try { ws.send(JSON.stringify({ type: 'ack', msg: 'connected', id })); } catch(e) {}

  ws.on('message', (data) => {
    let msg = null;
    try { msg = JSON.parse(data.toString()); } catch(e) { return; }

    // simple broadcast to all connected clients
    const payload = JSON.stringify(msg);
    let count = 0;
    for (const [cid, client] of clients) {
      if (client.readyState === client.OPEN) {
        try { client.send(payload); count++; } catch(e) {}
      }
    }

    // Throttled logging for forwarded messages
    if (msg && msg.type) {
      throttledLog(`[WS BCST] forwarded type=${msg.type} to ${count} clients`);
    }
  });

  ws.on('close', (code, reason) => {
    clients.delete(id);
    console.log(`WS client disconnected: ${id} code=${code} reason=${String(reason).slice(0,80)}`);
  });

  ws.on('error', (err) => {
    console.error('WS error for client', id, err && err.message ? err.message : err);
  });
});

// --- graceful shutdown ---
function shutdown() {
  console.log('Shutting down server...');
  try {
    wss.clients.forEach((c) => { try { c.close(); } catch(e) {} });
  } catch(e) {}
  try { server.close(() => process.exit(0)); } catch(e) { process.exit(0); }
}
process.on('SIGINT', shutdown);
process.on('SIGTERM', shutdown);

// start listening
server.listen(PORT, HOST, () => {
  console.log(`HTTPS + WSS server running at: https://${HOST}:${PORT}`);
  console.log(`Serving files from: ${ROOT}`);
  console.log(`Proxying MJPEG from http://${UPSTREAM_HOST}:${UPSTREAM_PORT}/stream -> https://${HOST}:${PORT}/camera/stream`);
});
