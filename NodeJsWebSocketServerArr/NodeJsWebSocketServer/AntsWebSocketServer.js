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
var clientsArr;
var commandsArr = [];

//when browser sends get request, send html file to browser. viewed at http://localhost:30000
app.get('/cmds', function(req, res) {
	
	if (!commandsArr) {
		commandsArr.forEach(element => {
			console.log(element);				
		});
	}
});

function noop() {}
function heartbeat() {
  this.isAlive = true;
}


function processCommand(input)
{
	var commandToSend ="";
			var strCmd = input;//d.toString().trim();
			var numOfSteps =strCmd.substring(1,2);
			if (numOfSteps == "")
				{	
					numOfSteps="0"
				}
				console.log("numOfSteps: " +numOfSteps);
			switch(strCmd.substring(0,1))
			{
				case 'f':
							commandToSend ="AC_0_GF"
					break;		
					
				case 'b':
						commandToSend="AC_0_GB"
						break;

				case 'r':
						commandToSend="AC_0_TR"
						break;

				case 'l':
						commandToSend="AC_0_TL"
						break;
				case '<':
						commandToSend="AC_0_BTL"
						break;
				case '>':
						commandToSend="AC_0_BTR"
						break;

				case 's':
						commandToSend="AC_0_S"
						break;
		

			}

			commandToSend = commandToSend.replace('0',numOfSteps);
			console.log("Sending " + commandToSend + " To the antBot");
			return commandToSend;

}


wss.on('connection',function(ws,req){
	//clientsArr.push(ws);
	console.log("Type Command:")
	console.log("f/ b/ r/ l/ tr/ tl/ s--> 0-9")
	var stdin = process.openStdin();
	stdin.addListener("data", function(d) {
		// note:  d is an object, and when converted to a string it will
		// end with a linefeed.  so we (rather crudely) account for that  
		// with toString() and then trim() 
		console.log("you entered: [" + 
			d.toString().trim() + "]");
			var strCmd = d.toString().trim();
			var commandToSend= processCommand(strCmd);
			console.log(commandToSend);
		ws.send(commandToSend);
	  });
	ws.on('message',function(message){
		//const msg = JSON.parse(message); //TODO: what if the message is not a JSON?...
		console.log("this is a message from client: " + message);
		//ws.send('GF');
		
		//ws.send("Server: got message: '" + message + "'"); //send to client where message is from
	});
	

	ws.on('close', function(){
		console.log("lost one client");
	});

	
	ws.send("hi new client!");
	console.log("new client connected");
});

console.log("Starting server on port 3000");
server.listen(3000);
console.log("Type Command:")
console.log("f/ b/ r/ l/ tr/ tl/ s--> 0-9")
var stdin = process.openStdin();
stdin.addListener("data", function(d) {
	console.log("you entered: [" + 
		d.toString().trim() + "]");
		var strCmd = d.toString().trim();
		if	(strCmd == "cmds") 
		{
			console.log("list of commands waiting to be sent:")
			commandsArr.forEach(function(element, index) {
			console.log('%d: %s', index +1, element);
				
			});
		}
		else
		{
			var commandToSend= processCommand(strCmd);
			commandsArr.push(commandToSend);
			console.log(commandToSend);
		}	
  });