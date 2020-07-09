
s.boot
s.quit

/*
 The OSC comunication is allowed by Delicode NI Mate
 - activate Skeleton OSC Enabled;
 - keep only hands joint and rename it with /Left_Hand and /Right_Hand;
 - Set smoothing and confidence to 50% to riduce mapping errors;
 - Set coordinates origin to Screen.

Bus list:
- busRight: right hand bus (3 channel) [control]
- busLeft: left hand bus (3 channel) [control]
- thBus: result of theremin SynthDef operation [audio]
- bpBus: result of bpFilter SynthDef operation [audio]
- ampBus: result of \analysis SynthDef Amplitude operation [control]
- freqBus: result of \analysis SynthDef Pitch operation [control]
- inBus: audio in [audio]
*/

/*
TODO:
*/


// -------------------------------------------------
// 0a. SYNTHDEF
// -------------------------------------------------

(
// Play the audio buffer in outBus bus
SynthDef(\playerBuffer, { |outBus = 0, audio|
	Out.ar (outBus, PlayBuf.ar(1, audio, loop:1));
	//Out.ar(outBus, SoundIn.ar);
}).add;

// Play the signal in inBus in out channel
SynthDef(\player, { |inBus, out = 0|
	Out.ar(out, In.ar(inBus))
}).add;


// Generate a wave enveloped
SynthDef(\generate, { | freq, out|
	var x = freq + MouseY.kr(15,25);
	//var sound = Pulse.ar(freq.midicps) * EnvGen.kr(Env.perc, 1, 0.2, doneAction:2);
	var sound = LFCub.ar(freq.midicps) * EnvGen.kr(Env.perc, 1, 0.2, doneAction:2); // A sine like shape made of two cubic pieces
	Out.ar(out,sound);
}).add;


// Read the signal in inBus, estimate pitch and amplitude and dump it in two different buses (ampBus and freqBus)
SynthDef(\analysis, { |inBus, ampBus, freqBus|
	var amp, freq, hasFreq, sin, sig;
	sig = In.ar(inBus);
	amp = Lag.kr(Amplitude.kr(sig)); // Amplitude generate a control signal
	#freq, hasFreq = Pitch.kr(sig); // freq: estimane frequency, hasFreq: tells whether a pitch was found

	Out.kr(ampBus, amp);
	Out.kr(freqBus, freq);
}).add;

/*
Simulate a theremin, in which the bFreq frequency is changed by another frequency that depends on the hand position
Frequency (f) changes based on right hand x-axis coord (ctrr)
Amplitude (amp) changes based on right hand y-axis coord (ctrl)

Input:
- inBus: input bus (signal);
- outBus: output bus;
- ctrl, ctrr: hands position (3-dimensional array) (from bus);
- bAmp: amplitude value (from bus);
- bFreq: frequency value (from bus);
Output:
- signal audio in outBus
*/
SynthDef(\theremin, { |inBus, outBus = 0, ctrl = #[0,0,0], ctrr = #[0,0,0], bAmp, bFreq|
	var sig, f, amp, audio;	 // sig: input signal, f: frequency, amp: amplitude, audio: audioOut

	sig = In.ar(inBus);

	//Un-comment to use mouse control
	//f = MouseX.kr(50, 20000,1); // warp = 1 means exponential mapping
	//amp = MouseY.kr(0,1);

	f = LinExp.kr (ctrr[0],0,1,200,10000);
	amp = LinLin.kr(ctrl[1],0,1,3,0);

	audio = SinOsc.ar(f, mul: amp) * Saw.ar(bFreq, mul: bAmp);
	//audio = SinOsc.ar(f, mul: amp) * LFSaw.ar(bFreq, mul: bAmp);
	//audio = SinOsc.ar(f, mul: amp) * LFPulse.ar(bFreq, mul: bAmp);

	Out.ar(outBus, audio);
}).add;


/*
This SynthDef apply a band-pass filter on signal in input (inBus)
The cutoff frequency is set by left hand x-position (from 50 to 20000 Hz)
The reciprocal of Q (cutoff / bandwith) depends by the euclidean distance between the hands

Input:
- inBus: input bus;
- outBus: output bus;
- ctrl, ctrr: hands position (array)
Output:
- signal audio in outBus
*/
SynthDef(\BPfilter, { |inBus, outBus = 0, ctrl = #[0,0,0], ctrr = #[0,0,0]|
	var rq, sig, freq, audio;

	sig = In.ar(inBus);

	//Un-comment to use mouse control
	//freq = MouseX.kr(50, 20000);
	//rq = MouseY.kr(0.001,1,1);


	freq = LinLin.kr(ctrl[0],0,1,400,20000);
	rq = ((ctrr[0] - ctrl[0]).squared + (ctrr[1] - ctrl[1]).squared).sqrt;
	rq = rq.linexp(0,1,0.0001,1);

	audio = BPF.ar(sig,freq, rq); // BP filter, rq: reciprocal of q (f_cutoff/bandwith)
	Out.ar(outBus, audio);
}).add;
)


// -------------------------------------------------
// 0b. LOAD SOUND IN BUFFER BUFFER + BUS
// -------------------------------------------------

(
// Allow to load an array of audio signals from path
~load = {|path, nameList|
	var bufArr = Array.fill(nameList.size, { |i|
		Buffer.readChannel(s, path ++ nameList[i] ++ ".wav", channels:[0]).normalize;
	});
	bufArr;
};

~path = "/Users/aleclock/Desktop/uni/EAM/valle/progetto/src/";
//~nameList = "Gold hiroshima".split($ );
//~nameList = "Gold".split($ );
~nameList = "hiroshima".split($ );
~buffArr = ~load.(~path, ~nameList);

// Create a list of buses
~inBus = Bus.audio(s,1);
~thBus = Bus.audio(s,1);
~bpBus = Bus.audio(s,1);
~ampBus = Bus.control(s,1);
~freqBus = Bus.control(s,1);
)


// -------------------------------------------------
// 1. ACTIVATE KINECT COMUNICATION
// -------------------------------------------------

(
~busRight = Bus.control(s,3); // Right hand
~busLeft = Bus.control(s,3); // Left hand
~address = NetAddr("127.0.0.1"); // local machine


/*
Function that allow to control a OSC message (Open Sound Control)

Input:
- name: name of OSC message
- bus: bus in which dump the message
- path: OSC message
- address: IP address of the comunication
- port: port of the comunication
*/
~defineOSC = { |name, bus, path, address, port = 7000|
	OSCdef(
	name,
	{
		|msg, time, addr, recvPort|
		//msg.postln; // Debug control
		//Set list of values in each channel of the control bus.
		bus.setnSynchronous([msg[1], msg[2], msg[3] + 0.3]); // Value range: [0,1],[0,1],[0,2+],
	},
	path,address, port
	);
};

~defineOSC.(\rh_xyz, ~busRight,"/Right_Hand", ~address, 7000);
~defineOSC.(\lh_xyz, ~busLeft,"/Left_Hand", ~address, 7000);

OSCdef(\rh_xyz).enable;
OSCdef(\lh_xyz).enable;
)

// -------------------------------------------------
// 2. USER INTERFACE
// -------------------------------------------------

(
var w_width = 720;
var w_height = 480;
var u_height = 440;
var c_dimension = 50; // controller dimension
var offsetX = 25;
var offsetY = (w_height-u_height)/2;
var x_space = 10;
var circleSize = 25;

var start, bp, th, bp_input, play; // Controls variables

// List of colors
var cGreen = Color.fromHexString("#81B29A");
var cTerraCotta = Color.fromHexString("#E07A5F");
var cFlame = Color.fromHexString("#EB5E28");
var cChampagne = Color.fromHexString("#F2CC8F");
var cBlack = Color.fromHexString("#252422");
var cBlackOlive = Color.fromHexString("#403D39");
var cFlorealWhite = Color.fromHexString("FFFCF2");
var cSteelTeal = Color.fromHexString ("#718F94");

w = Window("Theremin",Rect(0,0,w_width,w_height)).front;
w.background = cBlack;

/* Button used to start/stop the audio in in inBus
State 0 -> no signal
State 1 -> generate audio signal (\generate SynthDef)
State 2 -> play buffer signal (\playerBuffer SynthDef)
*/
start = Button(w, Rect(offsetX,offsetY,c_dimension,c_dimension))
.states_([
	["Start", cFlorealWhite, cBlackOlive],
	["Buffer", cFlorealWhite, cSteelTeal],
	["Stop", cFlorealWhite, cGreen]
])
.action_({ |butt|
	case
	{butt.value == 0} { if (~src.notNil, {~src.free; ~out.free; ~src = nil; ~out = nil })}
	{butt.value == 1} {
		var pitches = [0,2,3,5,7,8,10] + 60; // scala minore naturale
		var num = pitches.size ;
		var times = Array.geom(num, 1, 1.01) ;

		// Create an array of Routine, each one reproduce infine times \generate SynthDef
		~src = Array.fill(num, { |i|
			Routine({
				inf.do ({ times[i].wait;
					Synth(\generate, [\freq, pitches[i], \out, ~inBus]);
				});
			});
		});

		~src.do({arg rt ; rt.play}); // Play each Routine in ~src array
		~out = Synth(\player, [\inBus, ~inBus], addAction: \addAfter);
	}
	{butt.value == 2} {
		~src.do({arg rt ; rt.stop}); // Stop each Routine in ~src array
		~src = Synth(\playerBuffer, [\outBus, ~inBus,\audio, ~buffArr.choose], addAction: \addToHead); // ~src became a Synth (\playBuffer)
	}
});

/* Button that allow to switch on /off theremin
State 0 -> Theremin off (free all variables)
State 1 -> Theremin on (create Synth based on \analysis and \theremin SynthDef)
*/
th = Button(w, Rect(3*offsetX + c_dimension,offsetY,c_dimension,c_dimension))
.states_([
	["TH", cFlorealWhite, cBlackOlive],
	["TH", cFlorealWhite, cGreen]
])
.action_({ |butt|
	case
	{butt.value == 0} { if (~thSynth.notNil, {~thSynth.free; ~analysis.free; ~thSynth = nil; ~analysis = nil })}
	{butt.value == 1} {
		~analysis = Synth(\analysis,[\inBus, ~inBus, \freqBus, ~freqBus,\ampBus, ~ampBus], addAction: \addAfter);

		~thSynth = Synth(\theremin,[\inBus, ~inBus, \outBus,~thBus], addAction: \addAfter);
		~thSynth.map (\ctrl, ~busLeft, \ctrr, ~busRight, \bAmp, ~ampBus, \bFreq, ~freqBus);
	}
});


/* Button that allow to activate or de-activate the band-pass effect
State 0 -> BP filter off (free all variables)
State 1 -> BP filter on (create Synth based on \BPfilter SynthDef)
*/
bp = Button(w, Rect(3 * offsetX + (2 * c_dimension + x_space),offsetY,c_dimension,c_dimension/2 - 5))
.states_([
	["BPF", cFlorealWhite, cBlackOlive],
	["BPF", cFlorealWhite, cGreen]
])
.action_({ |butt|
	case
	{butt.value == 0} { if (~bpSynth.notNil, {~bpSynth.free; ~bpSynth = nil})}
	{butt.value == 1} {
		~bpSynth = Synth(\BPfilter,[\inBus, ~inBus, \outBus, ~bpBus], addAction: \addAfter);
		~bpSynth.map (\ctrl, ~busLeft, \ctrr, ~busRight, \ctrr);
	}
});

/* Button used to choise in which bus (inBus or thBus) apply the \BPfilter SynthDef
State 0 -> BP filter applied to ~inBus bus (original signal)
State 1 -> BP filter applied to ~thBus bus (result of theremin effect)
*/
bp_input = Button(w, Rect(3*offsetX + (2 *c_dimension + x_space),2 * offsetY + c_dimension/2 + 5,c_dimension,c_dimension/2 - 5))
.states_([
	["IN", cFlorealWhite, cBlackOlive],
	["TH", cFlorealWhite, cTerraCotta]
])
.action_({ |butt|
	case
	{butt.value == 0} { if (~bpSynth.notNil, {~bpSynth.set(\inBus, ~inBus)})}
	{butt.value == 1} { if (~bpSynth.notNil, {~bpSynth.set(\inBus, ~thBus)})}
});

/* Button used to reproduce in output the signal of a specific bus (inBus, thBus or bpBus)
State 0 -> Reproduce the signal in ~inBus bus (original signal)
State 1 -> Reproduce the signal in ~thBus bus (theremin result)
State 2 -> Reproduce the signal in ~bpBus bus (BP filter result)
*/
play = Button(w, Rect(5 * offsetX + (3 * c_dimension + x_space),offsetY,c_dimension,c_dimension))
.states_([
	["IN", cFlorealWhite, cBlackOlive],
	["TH", cFlorealWhite, cTerraCotta],
	["BP", cFlorealWhite, cSteelTeal]
])
.action_({ |butt|
	case
	{butt.value == 0} { if (~out.notNil, {~out.set(\inBus, ~inBus)})}
	{butt.value == 1} { if (~out.notNil, {~out.set(\inBus, ~thBus)})}
	{butt.value == 2} { if (~out.notNil, {~out.set(\inBus, ~bpBus)})}
});

// Define a UserView in which draw hands position
u = UserView(w, Rect(0,80,w_width, u_height)).front;

~dur = 0.1; // Refresh rate of UserView

// Initialize hands position
~rHand = [0,0,0];
~lHand = [0,0,0];

// Function that draw in UserView a oval (input x,y,radius) with color color
~drawOval = { arg x = 0, y = 0, radius = 10, color = Color.white;
	Pen.fillColor_(color);
	Pen.addOval(Rect(x,y,radius, radius));
	Pen.fill;
};

// Drawing function called when declared and with .refresh attribute
u.drawFunc_({
	Pen.fillColor_(cFlame);
	Pen.addRect(Rect(20, 0, u.bounds.width-40, 1));
	Pen.perform(\fill);

	~drawOval.(~lHand[0].linlin(0,1,0,720),~lHand[1].linlin(0,1,0,400), ~lHand[2].linlin(0,2,2,1) * circleSize, cChampagne);
	~drawOval.(~rHand[0].linlin(0,1,0,720),~rHand[1].linlin(0,1,0,400), ~rHand[2].linlin(0,2,2,1) * circleSize, cSteelTeal);
});

w.refresh;
u.clearOnRefresh_(true); // In questo modo l'interfaccia non viene sovrascritta

// Routine used to refresh UI
r = Routine({
	inf.do{
		~busLeft.get{ |v|  ~lHand = v;};
		~busRight.get{ |v| ~rHand = v;};

		u.refresh;
		~dur.wait;
	};
});

r.reset.play(AppClock);
)



~freqBus.scope
~ampBus.scope

~busRight.scope
~inBus.scope
~thBus.scope
~bpBus.scope

s.freqscope
s.scope