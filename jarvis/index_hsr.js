//var APP_ID = 'amzn1.ask.skill.15a7b843-cdfe-46d1-9086-ccb695bcb189'; // init skill
var APP_ID = 'amzn1.ask.skill.1b691658-7e9d-41b1-a9d5-c00eb91d57a4'; // hsr c3
var Alexa = require('alexa-sdk');

var ROSLIB = require('roslib');
var EMITTER = require('eventemitter2');

var HELP_REPROMPT = "What can I help you with?";
var HELP_MESSAGE = "You can say forward, backward turn left or turn right... What can I help you with?";
var STOP_MESSAGE = "Goodbye!";
var RECIPE_NOT_FOUND_MESSAGE = "Please try again.";
var RECIPE_NOT_FOUND_REPROMPT = "What else can I help with?";

//ROS functionality
var ros = new ROSLIB.Ros({
        url: 'ws://localhost:9090'
    });

ros.on('connection', function () {
    console.log('Connected to websocket server.');
});

ros.on('error', function (error) {
    console.log('Error connecting to websocket server: ', error);
});

ros.on('close', function () {
    console.log('Connection to websocket server closed.');
});

var hsrTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/zavengers/jarvis',
    messageType: 'std_msgs/String'
});                   

//Amazon Alexa handlers
var handlers = {
    'LaunchRequest': function () {
       hsrTopic.advertise();
       this.emit(':tell', 'Ready to command HSR');
   },
   'switch_object': function () {
     var command_array = ["switch"]

        if(!this.event.request.intent.slots.object.value){
            // No request
            var obj = "null";
        } else if (this.event.request.intent.slots.object.resolutions.resolutionsPerAuthority[0].status.code == "ER_SUCCESS_NO_MATCH"){
            // Error - bad request - doesn't match any of the objects listed in the alexa skill
            this.emit(':tell', 'Bad switch request');
            return;
        } else {
            // Valid object request
            var obj = this.event.request.intent.slots.object.resolutions.resolutionsPerAuthority[0].values[0].value.name;
        }

        command_array.push(obj);
        var command = command_array.join(" ");
        var track_msg = new ROSLIB.Message({data: command});
        hsrTopic.publish(track_msg)
	 	// Send alexa response
		this.emit(':tell', 'Switching to ' + obj);
    },
   'grasp_configuration': function () {
     var command_array = ["config"]

        if(!this.event.request.intent.slots.grasp_type.value){
            // No request
            var fb = "null";
        } else if (this.event.request.intent.slots.grasp_type.resolutions.resolutionsPerAuthority[0].status.code == "ER_SUCCESS_NO_MATCH"){
            // Error - bad request - doesn't match any of the objects listed in the alexa skill
            this.emit(':tell', 'Bad grasp configuration request');
            return;
        } else {
            // Valid grasp request
            var fb = this.event.request.intent.slots.grasp_type.resolutions.resolutionsPerAuthority[0].values[0].value.name;
        }

        command_array.push(fb);
        var command = command_array.join(" ");
        var track_msg = new ROSLIB.Message({data: command});
        hsrTopic.publish(track_msg)
	 	// Send alexa response
		this.emit(':tell', 'Using ' + fb + ' grasp configuration');
   },
   'task_feedback': function () {
     var command_array = ["task"]

        if(!this.event.request.intent.slots.feedback.value){
            // No feedback request
            var fb = "null";
        } else if (this.event.request.intent.slots.feedback.resolutions.resolutionsPerAuthority[0].status.code == "ER_SUCCESS_NO_MATCH"){
            // Error - bad feedback request - doesn't match any of the objects listed in the alexa skill
            this.emit(':tell', 'Bad task request');
            return;
        } else {
            // Valid feedback request
            var fb = this.event.request.intent.slots.feedback.resolutions.resolutionsPerAuthority[0].values[0].value.name;
        }

        command_array.push(fb);
        var command = command_array.join(" ");
        var track_msg = new ROSLIB.Message({data: command});
        hsrTopic.publish(track_msg)
	 	// Send alexa response
		this.emit(':tell', 'Grasp ' + fb);
   },
   'grab_object': function () {
     var command_array = ["grab"]

        if(!this.event.request.intent.slots.object.value){
            // No object request
            var obj = "null";
        } else if (this.event.request.intent.slots.object.resolutions.resolutionsPerAuthority[0].status.code == "ER_SUCCESS_NO_MATCH"){
            // Error - bad object request - doesn't match any of the objects listed in the alexa skill
            this.emit(':tell', 'Bad object request');
            return;
        } else {
            // Valid object request
            var obj = this.event.request.intent.slots.object.resolutions.resolutionsPerAuthority[0].values[0].value.name;
        }

        command_array.push(obj);
        var command = command_array.join(" ");
        var track_msg = new ROSLIB.Message({data: command});
        hsrTopic.publish(track_msg)
	 	// Send alexa response
		this.emit(':tell', 'Grabbing ' + obj);
    },
    'Unhandled': function () {
        this.emit(':tell', 'Intent Unknown');
    },
    'AMAZON.HelpIntent': function () {
        var speechOutput = HELP_MESSAGE;
        var reprompt = HELP_REPROMPT;
        this.emit(':ask', speechOutput, reprompt);
    },
    'AMAZON.CancelIntent': function () {
        this.emit(':tell', STOP_MESSAGE);
    },
    'AMAZON.StopIntent': function () {
        this.emit(':tell', STOP_MESSAGE);
    }
};

exports.handler = function (event, context, callback) {
    var alexa = Alexa.handler(event, context);
    alexa.APP_ID = APP_ID;
    alexa.registerHandlers(handlers);
    alexa.execute();
};
