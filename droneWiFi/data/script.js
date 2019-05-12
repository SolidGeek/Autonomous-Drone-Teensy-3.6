// Websocket object to be used
var websocket;

// Element references to modify DOM
var statusBar 	= document.getElementById("status");
var logElement 	= document.getElementById("log");
var sendRoll 	= document.getElementById("send_roll");
var sendPitch 	= document.getElementById("send_pitch");
var sendYaw 	= document.getElementById("send_yaw");
var sendOffset 	= document.getElementById("send_offset");
var emergency 	= document.getElementById("emergency");
var start 		= document.getElementById("start");
var save 		= document.getElementById("save");

var configRead = false;

// Interval in ms between each alive message
var aliveInterval = 100;


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

	setInterval(function() {

		if(websocket.readyState == 1){

			sendCommand( "ALIVE" );

		}

	}, aliveInterval);
}

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

}

function sendCommand( command, param = [] ){
	
	var gcode = command;

	if( param.length > 0 ){
		var parameters = param.map(e => e.name + e.value ).join(' ');
		gcode += " " + parameters;
	}

	if(websocket.readyState == 1){
		
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

	if( data.includes("TELEMETRY") ){

		var tlm = [];

		var items = data.split(" ");
		items.shift(); // Remove "TELEMETRY" from array
		items.pop(); // Remove empty last element

		for (var i in items) {
	    	tlm[i] = parseFloat( items[i] );
		}

		roll.append(Date.now(), tlm[0]);
		pitch.append(Date.now(), tlm[1]);
		// yaw.append(Date.now(), tlm[2]);

		document.getElementById('roll_value').value = tlm[0];
		document.getElementById('pitch_value').value = tlm[1];
		document.getElementById('yaw_value').value = tlm[2];

		altitude.append( Date.now(), tlm[3]);
		document.getElementById('alt_value').value = tlm[3];

		document.getElementById('voltage').value = tlm[4] + " V";

		// Motor outputs
		document.getElementsByName('motor1')[0].value = tlm[5];
		document.getElementsByName('motor2')[0].value = tlm[6];
		document.getElementsByName('motor3')[0].value = tlm[7];
		document.getElementsByName('motor4')[0].value = tlm[8];

	}

	else if( data.includes("CONFIG") ) {

		console.log( data );

		var settings = [];

		var items = data.split(" ");
		items.shift(); // Remove "CONFIG" from array

		for (var i in items) {
	    	settings[i] = parseFloat( items[i] );
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

		document.getElementsByName('alt_p')[0].value = settings[9];
		document.getElementsByName('alt_i')[0].value = settings[10];
		document.getElementsByName('alt_d')[0].value = settings[11];

		document.getElementsByName('hoverOffset')[0].value = settings[12];
		document.getElementById('hoverOffsetShow').value = settings[12];

		document.getElementsByName('offset1')[0].value = settings[13];
		document.getElementsByName('offset2')[0].value = settings[14];
		document.getElementsByName('offset3')[0].value = settings[15];
		document.getElementsByName('offset4')[0].value = settings[16];

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

function sameMinMaxRange(range) {	
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

function minMaxRange(range) {	
	var max = 0;
	var min = 0;

	if( range.max > max ){
		max = range.max;
	}

	if( range.min < min ){
		min = range.min;
	}

	return {min: min, max: max};
}

/* ---- AngleChart ---- */
var roll = new TimeSeries();
var pitch = new TimeSeries();
var yaw = new TimeSeries();

var angleCanvas = document.getElementById('angleChart');

var angleChart = new SmoothieChart({
	yRangeFunction:sameMinMaxRange,
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

angleChart.addTimeSeries(roll, { strokeStyle: '#ec1a1a' });
angleChart.addTimeSeries(pitch, { strokeStyle: '#01799e' });
angleChart.addTimeSeries(yaw, { strokeStyle: '#0e9a27' });

angleChart.streamTo(angleCanvas, 100);
 

/* ---- AltitudeChart ---- */
var altitude = new TimeSeries();

var altCanvas = document.getElementById('altChart');

var altChart = new SmoothieChart({
	millisPerPixel:10,
	grid:{
		fillStyle:'transparent',
		verticalSections:10,
		strokeStyle:'#c9c9c9'
	},
	labels:{fillStyle:'#000',fontSize:14},
	tooltip:true,
	responsive: true,
	yRangeFunction:minMaxRange,
});

altChart.addTimeSeries(altitude, { strokeStyle: '#000' });

altChart.streamTo(altCanvas, 100);

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

sendYaw.onclick = function()
{
	// Save the settings to variables
	var kp = document.getElementsByName('yaw_p')[0].value;
	var ki = document.getElementsByName('yaw_i')[0].value;
	var kd = document.getElementsByName('yaw_d')[0].value;

	// Send settings over WebSocket
	sendCommand( "YAW", [
		{name: "P", value: kp},
		{name: "I", value: ki},
		{name: "D", value: kd},
	] );
};

sendOffset.onclick = function(){

	// Save the settings to variables
	var hoverOffset = document.getElementsByName('hoverOffset')[0].value;
	var offset1 = document.getElementsByName('offset1')[0].value;
	var offset2 = document.getElementsByName('offset2')[0].value;
	var offset3 = document.getElementsByName('offset3')[0].value;
	var offset4 = document.getElementsByName('offset4')[0].value;

	// Send settings over WebSocket
	sendCommand( "OFFSET", [
		{name: "H", value: hoverOffset},
		{name: "A", value: offset1},
		{name: "B", value: offset2},
		{name: "C", value: offset3},
		{name: "D", value: offset4},
	] );
}

emergency.onclick = function()
{
	// Send settings over WebSocket
	sendCommand( "STOP" );
};

start.onclick = function(){
	sendCommand( "START" );
}

save.onclick = function()
{
	// Send settings over WebSocket
	sendCommand( "SAVE" );
};

function openTab(evt, name) {
	// Declare all variables
	var i, tabcontent, tablinks;

	// Get all elements with class="tabcontent" and hide them
	tabcontent = document.getElementsByClassName("tabcontent");
	for (i = 0; i < tabcontent.length; i++) {
		tabcontent[i].style.display = "none";
	}

	// Get all elements with class="tablinks" and remove the class "active"
	tablinks = document.getElementsByClassName("tablinks");
	for (i = 0; i < tablinks.length; i++) {
		tablinks[i].className = tablinks[i].className.replace(" active", "");
	}

	// Show the current tab, and add an "active" class to the button that opened the tab
	document.getElementById(name).style.display = "block";
	evt.currentTarget.className += " active";
}

document.getElementById("defaultOpen").click();