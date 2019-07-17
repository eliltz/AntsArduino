var bodyParser = require("body-parser");
const express = require('express'); //express framework to have a higher level of methods
const app = express(); //assign app variable the express class/method
var http = require('http');
var path = require("path");
app.use(bodyParser.urlencoded({ extended: false }));
app.use(bodyParser.json());
const server = http.createServer(app);//create a server


app.use(express.static('pages'));


//node_modules\nipplejs
//path.join(__dirname + '/index.html')


/**********************websocket setup**************************************************************************************/
//var expressWs = require('express-ws')(app,server);
const WebSocket = require('ws');
const wss = new WebSocket.Server({ server });

//when browser sends get request, send html file to browser. viewed at http://localhost:30000
app.get('/', function(req, res) {
	res.sendFile(index.html);
});

function noop() {}
function heartbeat() {
  this.isAlive = true;
}



/* 
const interval = setInterval(function ping() {
  wss.clients.forEach(function each(ws) {
//    if (ws.isAlive === false) return ws.terminate();

	ws.send("HI!\n");
    ws.isAlive = false;
    ws.ping(heartbeat);
	
	console.log(ws.isAlive?"alive":"dead");
		
 });
}, 10000);

*/

//var oldPosX=0;
//var oldPosY=0;
wss.on('connection',function(ws,req){
	ws.on('message',function(message){
		const msg = JSON.parse(message); //TODO: what if the message is not a JSON?...
		if (msg.type === 'joystickMessage'){
			
			//if  ( Math.abs (oldPosX - msg.data.position.x) > 2 || Math.abs(oldPosY - msg.data.position.y) > 2 ) { //if position changed
				//oldPosX = Math.floor(msg.data.position.x);
				//oldPosY = Math.floor(msg.data.position.y);

				wss.clients.forEach(function(client){ //broadcast incoming message to all clients (s.clients)
					if(client!=ws && client.readyState ){ //except to the same client (ws) that sent this message


						var power = Math.sin(msg.data.angle.radian) * msg.data.distance;
						var percent = Math.cos(msg.data.angle.radian) * msg.data.distance;

						percent = percent/100 + 0.5

						//console.log( power + " " + percent);

						const multiplier = 9;

						//figure out movment mode:
						if (Math.abs(power)<10){ 
							if (percent<0.6 && percent>0.4){ 	//joystick in middle position-> no significant power nor direction - just stop.
								client.send('0 0 ');
							}else {													//no significant power - turn by counter rotation of tracks (on the spot)
								client.send(-1*Math.floor(multiplier*(percent-0.5)*50) + ' ' + Math.floor(multiplier*(percent-0.5)*50));
							} 
						}else{ 								
							if(percent<0.6 && percent>0.4){ 	//no significant steering. drive straight.
								client.send(Math.floor(multiplier*power*percent) + ' ' + Math.floor(multiplier*power*percent));
							}
							else {														//significant power and steering - turn with wheels/track power ratio
								client.send(Math.floor(multiplier*power*(1-percent)) + ' ' + Math.floor(multiplier*power*percent));
							}
						}
					}
				});
			//}

		}
		else{

		}
		
		


		
		//ws.send("Server: got message: '" + message + "'"); //send to client where message is from
	});
	

	ws.on('close', function(){
		console.log("lost one client");
	});

	
	ws.send("hi new client!");
	console.log("new client connected");
});

server.listen(3000);
