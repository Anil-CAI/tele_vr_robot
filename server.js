const express = require('express');
const WebSocket = require('ws');
const path = require('path');

const app = express();

// serve files + node_modules
app.use(express.static(__dirname));
app.use('/node_modules', express.static(path.join(__dirname, 'node_modules')));

// default route → VR UI
app.get('/', (req, res) => {
  res.sendFile(path.join(__dirname, 'index_vr.html'));
});

const server = app.listen(8080, () => {
  console.log("HTTP+WS server running at http://localhost:8080");
});

// WebSocket server
const wss = new WebSocket.Server({ server });

wss.on('connection', ws => {
  console.log("WS connected");

  ws.on('message', msg => {
    console.log("WS msg:", msg.toString());

    // BROADCAST to ALL clients (browser + Python bridge)
    wss.clients.forEach(c => {
      if (c.readyState === WebSocket.OPEN) {
        c.send(msg.toString());
      }
    });
  });
});

