// Websocket object to be used
var websocket;

// Element references to modify DOM
var statusBar 	= document.getElementById("status");
var logElement 	= document.getElementById("log");
var sendRoll 	= document.getElementById("send_roll");
var sendPitch 	= document.getElementById("send_pitch");
var sendYaw 	= document.getElementById("send_yaw");
var emergency 	= document.getElementById("emergency");
var save 		= document.getElementById("save");


var configRead = false;

// Min interval in ms between each command 
var wsInterval = 500;


window.onload = function() {
	// Initiate the websocket connection
	initWebSocket();

	// Always try to reinitiate the Websocket connection
	setInterval(function() {

		if( websocket.readyState != 1 ){
			initWebSocket();
		}else{

			if( configRead == false ){
				sendCommand( "CONFIG" ); 
			}
			
		}

	}, 1500);
}

/* Keep the websocket connection alive 
var websocketInterval = function() {

	if(websocket.readyState == 1){

		sendCommand( "ALIVE" );

	}

	// Set next timeout
    setTimeout(websocketInterval, wsInterval);
}*/





function initWebSocket()
{

	if( websocket ) websocket.close();
	
	websocket = new WebSocket('ws://192.168.4.1:81/');

	addToLog("Connecting...");
	setStatus("Connecting", "primary")

	websocket.onopen = function(event) { onWsOpen(event) };
	websocket.onclose = function(event) { onWsClose(event) };
	websocket.onmessage = function(event) { onWsMessage(event) };

	window.addEventListener('beforeunload', function() {
		websocket.close();
	});

	// setTimeout(websocketInterval, wsInterval);

}

function sendCommand( command, param = [] ){
	
	var gcode = command;

	if( param.length > 0 ){
		var parameters = param.map(e => e.name + e.value ).join(' ');
		gcode += " " + parameters;
	}

	if(websocket.readyState == 1){
		console.log( gcode );
		websocket.send( gcode + " " );
	}
}

function onWsOpen(event) {
	addToLog("Websocket connection established");
	setStatus("Connected", "success");
}

function onWsClose(event) {
	addToLog("Websocket connection lost");
	setStatus("No connection", "danger")
}

// Whenever a message is received from the ESP
function onWsMessage(event) {

	var data = event.data;

	if( data.includes("POS") ){

		console.log( data );

		var tlm = [];

		// Read data about position and update current positions
		var items = data.split(" ");
		items.shift(); // Remove "ANGLE" from array

		for (var i in items) {
	    	tlm[i] = parseFloat( items[i].substring(1, items[i].length) );
		}

		roll.append(Date.now(), tlm[2]);
		pitch.append(Date.now(), tlm[1]);
		// yaw.append(Date.now(), tlm[0]);

		document.getElementById('roll_value').value = tlm[2];
		document.getElementById('pitch_value').value = tlm[1];
		document.getElementById('yaw_value').value = tlm[0];
		document.getElementById('alt_value').value = tlm[3];
	}

	else if( data.includes("SPEED") ){

		var speeds = [];

		// Read data about position and update current positions
		var items = data.split(" ");

		items.shift(); // Remove "ANGLE" from array

		for (var i in items) {
	    	speeds[i] = parseFloat( items[i].substring(1, items[i].length) );
		}

		document.getElementsByName('motor1')[0].value = speeds[0];
		document.getElementsByName('motor2')[0].value = speeds[1];
		document.getElementsByName('motor3')[0].value = speeds[2];
		document.getElementsByName('motor4')[0].value = speeds[3];

	}

	else if( data.includes("CONFIG") ) {

		var settings = [];

		// Read data about position and update current positions
		var items = data.split(" ");

		items.shift(); // Remove "ANGLE" from array

		for (var i in items) {
	    	settings[i] = parseFloat( items[i].substring(1, items[i].length) );
		}

		document.getElementsByName('roll_p')[0].value = settings[0];
		document.getElementsByName('roll_i')[0].value = settings[1];
		document.getElementsByName('roll_d')[0].value = settings[2];
		
		document.getElementsByName('pitch_p')[0].value = settings[3];
		document.getElementsByName('pitch_i')[0].value = settings[4];
		document.getElementsByName('pitch_d')[0].value = settings[5];

		document.getElementsByName('yaw_p')[0].value = settings[6];
		document.getElementsByName('yaw_i')[0].value = settings[7];
		document.getElementsByName('yaw_d')[0].value = settings[8];

		configRead = true;

	}
}

function addToLog( data ){

	var options = { };
	var now = new Date();

	if(logElement.value != ""){
		logElement.value += "\n";
	}

	logElement.value += now.toLocaleTimeString('en-GB', options) + ": " + data;
	logElement.scrollTop = logElement.scrollHeight;

}

function setStatus( text, type ){
	statusBar.className = "badge badge-" + type;
	statusBar.innerHTML = text;
}




// Definition of objects to hold realtime data for the chart
var roll = new TimeSeries();
var pitch = new TimeSeries();
var yaw = new TimeSeries();

// Get the element object with the ID angleChart
var angleCanvas = document.getElementById('angleChart');


function myYRangeFunction(range) {	
	var max = 0;
	var min = 0;

	if( range.max > max ){
		max = range.max;
	}

	if( Math.abs(range.min) > max ){
		max = Math.abs(range.min);
	}

	min = -max;
	max = max;

	return {min: min, max: max};
}

// Setup the chart settings
var angleChart = new SmoothieChart({
	yRangeFunction:myYRangeFunction,
	millisPerPixel:10,
	grid:{
		fillStyle:'transparent',
		verticalSections:10,
		strokeStyle:'#c9c9c9'
	},
	labels:{fillStyle:'#000',fontSize:14},
	tooltip:true,
	responsive: true
});

// Assign each of the dataseries to their own graph on the chart, with each their different color
angleChart.addTimeSeries(roll, { strokeStyle: '#ec1a1a' });
angleChart.addTimeSeries(pitch, { strokeStyle: '#01799e' });
// angleChart.addTimeSeries(yaw, { strokeStyle: '#0e9a27' });

// Tell the chart object to begin streaming of data to the HTML canvas with a delay of 100ms
angleChart.streamTo(angleCanvas, 100);


sendRoll.onclick = function()
{
	// Save the settings to variables
	var kp = document.getElementsByName('roll_p')[0].value;
	var ki = document.getElementsByName('roll_i')[0].value;
	var kd = document.getElementsByName('roll_d')[0].value;

	// Send settings over WebSocket
	sendCommand( "ROLL", [
		{name: "P", value: kp},
		{name: "I", value: ki},
		{name: "D", value: kd},
	] );
};

sendPitch.onclick = function()
{
	// Save the settings to variables
	var kp = document.getElementsByName('pitch_p')[0].value;
	var ki = document.getElementsByName('pitch_i')[0].value;
	var kd = document.getElementsByName('pitch_d')[0].value;

	// Send settings over WebSocket
	sendCommand( "PITCH", [
		{name: "P", value: kp},
		{name: "I", value: ki},
		{name: "D", value: kd},
	] );
};

emergency.onclick = function()
{
	// Send settings over WebSocket
	sendCommand( "STOP" );
};

save.onclick = function()
{
	// Send settings over WebSocket
	sendCommand( "SAVE" );
};