const http = require('http');
const fs = require('fs');
const path = require('path');
const WebSocket = require('ws');

const PORT = process.env.PORT || 5050;
const CAPTURES_DIR = path.join(__dirname, 'captures');

if (!fs.existsSync(CAPTURES_DIR)) {
  fs.mkdirSync(CAPTURES_DIR, { recursive: true });
}

const server = http.createServer((req, res) => {
  // Enable CORS
  res.setHeader('Access-Control-Allow-Origin', '*');
  res.setHeader('Access-Control-Allow-Methods', 'POST, GET, OPTIONS');
  res.setHeader('Access-Control-Allow-Headers', '*');

  if (req.method === 'OPTIONS') {
    res.writeHead(200);
    res.end();
    return;
  }

  if (req.method === 'POST' && req.url === '/upload') {
    const sessionId = req.headers['x-session-id'] || `session_${Date.now()}`;
    const filename = req.headers['x-filename'] || 'file.bin';

    const sessionDir = path.join(CAPTURES_DIR, sessionId);
    if (!fs.existsSync(sessionDir)) {
      fs.mkdirSync(sessionDir, { recursive: true });
    }

    const filePath = path.join(sessionDir, filename);
    const writeStream = fs.createWriteStream(filePath);

    let bytesReceived = 0;
    req.on('data', (chunk) => {
      bytesReceived += chunk.length;
    });

    req.pipe(writeStream);

    writeStream.on('finish', () => {
      const mb = (bytesReceived / (1024 * 1024)).toFixed(2);
      console.log(`[Captured] Synced: ${sessionId}/${filename} (${mb > 0.01 ? mb + ' MB' : (bytesReceived / 1024).toFixed(1) + ' KB'})`);
      res.writeHead(200, { 'Content-Type': 'application/json' });
      res.end(JSON.stringify({ success: true, path: filePath, bytes: bytesReceived }));
    });

    writeStream.on('error', (err) => {
      console.error(`[Error] Failed to write ${filename}:`, err);
      res.writeHead(500, { 'Content-Type': 'application/json' });
      res.end(JSON.stringify({ error: err.message }));
    });
    return;
  }

  if (req.method === 'GET' && req.url === '/status') {
    res.writeHead(200, { 'Content-Type': 'application/json' });
    res.end(JSON.stringify({ status: 'ready', capturesDir: CAPTURES_DIR }));
    return;
  }

  res.writeHead(404);
  res.end('Not Found');
});

const wss = new WebSocket.Server({ server });

wss.on('connection', (ws, req) => {
  const ip = req.socket.remoteAddress;
  console.log(`[WebSocket Connected] ${ip}`);

  ws.on('message', (message) => {
    // Broadcast incoming sensor packets from phone to all listeners (e.g. Python viewer)
    wss.clients.forEach((client) => {
      if (client !== ws && client.readyState === WebSocket.OPEN) {
        client.send(message);
      }
    });
  });

  ws.on('close', () => {
    console.log(`[WebSocket Disconnected] ${ip}`);
  });
});

server.listen(PORT, '0.0.0.0', () => {
  console.log(`====================================================`);
  console.log(`Capture & Live Stream Server active on port ${PORT}`);
  console.log(`HTTP Upload: http://0.0.0.0:${PORT}/upload`);
  console.log(`WebSocket:   ws://0.0.0.0:${PORT}`);
  console.log(`====================================================`);
});
