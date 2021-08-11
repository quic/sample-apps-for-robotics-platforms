const fs = require('fs');
const http = require('http');
const WebSocket = require('ws');

const server = http.createServer(function (req, res) {
	fs.readFile('/data/node/server.html',function (err, data){
		res.writeHead(200, {'Content-Type': 'text/html','Content-Length':data.length}); 
		res.write(data);
		res.end();   	
	});
});

const wss = new WebSocket.Server({ server });

wss.on('connection', function connection(ws) {
	console.log("connection received");
	ws.on('message', function incoming(message) {
		console.log('received: %s', message);
	});
});

server.listen(8080);