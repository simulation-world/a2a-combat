; ============================= ;
;       AIR-TO-AIR COMBAT       ;
; ============================= ;
; AUTHOR: Charlie Wang
; https://github.com/simulation-world
; ----------------------------- ;

globals
[
  total-missiles-fired
  total-missed
  total-hit

  unit-speed-factor
]

breed [planes plane]
breed [interceptors interceptor]
breed [launchers launcher]

planes-own
[
  enemy-color
  rcs
  speed
  target         ; Aircraft target
  has-target     ; Bool
  assigned-hdg   ;
  missiles       ; How many missiles on the aircraft
  active-missiles ; List
  current-snr
]

interceptors-own
[
  speed
  target
  detonation-time  ; How long missile will fly before self-detonate
  originator
  last-relative-bearing
  last-los-rate
  integral-term
  time-burned ; How long motor has burned for
]

to setup-unit-conversions
  if units = "metric"
  [
    set unit-speed-factor 1 / patch-scale
    show (word "unit-speed-factor " unit-speed-factor)
  ]
  if units = "imperial"
  [
    set unit-speed-factor (1 / patch-scale) / 1.944
    ;show (word "unit-speed-factor " unit-speed-factor)
  ]
end

to setup-red-planes
  create-planes starting-red
  [
    setxy random-xcor random-ycor
    set color red
    set shape "airplane"
    set rcs 3 ; m^2
    set has-target false
    set missiles 2
    set enemy-color blue
    set speed plane-speed * unit-speed-factor
    ;pen-down
  ]
end

to setup-blue-planes
  create-planes starting-blue
  [
    setxy random-xcor random-ycor
    set color blue
    set shape "airplane"
    set rcs 1 ; m^2
    set has-target false
    set missiles 2
    set enemy-color red
    set speed plane-speed * unit-speed-factor
  ]
end

to setup-launchers
  create-launchers 1
  [
    setxy 0 min-pycor
    set color blue
    set shape "truck"
  ]
end

to set-blue-target ; Choose the closest target which is not on my side
  ask planes with [color = blue]
  [
    if target = nobody or target = 0
    [
      set target min-one-of planes with [color != [color] of myself] [distance self]
      ;show (word "target " target ", exists? " (target != nobody) ", enemy-color: " enemy-color)
      if target != nobody [set has-target true]
    ]
  ]
end

to setup
  clear-all
  reset-ticks

  setup-unit-conversions
  setup-red-planes
  setup-blue-planes
  set-blue-target
  ;setup-launchers
end

; -----------------------------

to launch-AAM
  hatch-interceptors 1
  [
    set shape "default"
    set detonation-time (ticks + max-tof)
    set originator myself
    set target [target] of myself
    set speed [speed] of myself
    set time-burned 1
    pen-down
    ask myself [set active-missiles (active-missiles + 1)]
  ]
  set total-missiles-fired total-missiles-fired + 1
end

; --- RED ---
to move-red ; Move with random maneuvering
  fd speed
  if ticks mod 50 = 0 [set assigned-hdg (heading + ((random 21) - 10))]
  let turn-amount subtract-headings heading assigned-hdg
  if turn-amount > 0 [rt 1]
  if turn-amount < 0 [lt 1]
end

to operate-red
  ask planes with [color = red]
  [
    move-red

  ]
end

; --- BLUE ---
to move-blue
  if target != nobody  ; Only maneuver if there is a valid target
  [
    let turn-amount subtract-headings heading (towards target)
    ;show (word "heading: " heading ", towards target: " towards target ", subtract-headings: " turn-amount)
    if turn-amount > 0 [lt 1 + (turn-amount / 360)]
    if turn-amount < 0 [rt 1 + (turn-amount / 360)]
  ]
  fd speed
end

; Current radar roughly based on AN/APQ-153
to operate-radar ;[radar-type]
  let pt 75000 ; transmitted_power (W)
  let gt 28 ; antenna_gain_transmitter (dB)
  let gr 28 ; antenna_gain_receiver  (dB) (made up)
  let wavelength 299792458 / 9.3e9
  let Ts 200 ; system_noise_temperature (K)
  let B 3e6 ; receiver_bandwidth (Hz)
  let Nf 8.18 ; receiver_noise_figure (dB)
  let k 1.380649e-23 ; Boltzmann constant

  let potential-targets planes with [color != [color] of myself] in-cone 22 max-off-boresight-angle
  ;if show-sensor-volume [ask patches in-cone 22 max-off-boresight-angle [set pcolor green]]

  ask potential-targets [
  let snr ((pt * gt * gr * wavelength ^ 2 * rcs) / ((4 * pi) ^ 3 * (distance myself * 1000) ^ 4 * k * Ts * B * Nf))
    if self = [target] of myself [ask myself [set current-snr snr]] ;show (word "distance: " (distance myself * 1000) ", snr: " snr)
  ]
  if (target = nobody) or (not member? target potential-targets) [set current-snr 0]
end

to fire-control
  ifelse target != nobody  ; Only operate fire control if there is a valid target
  [
    let engageable-targets planes with [color != [color] of myself] in-cone R-max max-off-boresight-angle ; Max engagement range
    if member? target engageable-targets ; If target is within max engagement range
    [
      ;show (word "FIRE!")
      if active-missiles < 1 and can-engage-AAM [launch-AAM]
    ]
  ]
  [ set-blue-target ]
end

to fly-interceptors
  ; ----- MANEUVER -----
  if target != nobody  ; Only maneuver if there is a valid target
  [
    ; --- Calculate angles ---
    let absolute-bearing towards target
    let relative-bearing heading - absolute-bearing
    set relative-bearing (relative-bearing + 360) mod 360 ; Ensure value is between 0-360
    if relative-bearing >= 180 [set relative-bearing relative-bearing - 360] ; Adjust the relative bearing to be between -180 to 180

    let los-rate (relative-bearing - last-relative-bearing)
    set last-relative-bearing relative-bearing
    ;show (word "absolute bearing: " absolute-bearing ", relative bearing: " relative-bearing ", LOS rate: " los-rate)

    ; --- Control actuation ---
    ; Always directly turn towards target's current position (velocity pursuit)
    ;let turn-amount subtract-headings heading (towards target)
    ;if turn-amount > 0 [lt 1 + (turn-amount / 360)]
    ;if turn-amount < 0 [rt 1 + (turn-amount / 360)]
    if los-rate > max-AAM-turn-rate [set los-rate max-AAM-turn-rate]
    if los-rate < (max-AAM-turn-rate * -1) [set los-rate (max-AAM-turn-rate * -1)]

    ; Use PID controller to mainatin LOS rate
    set integral-term (integral-term + Ki * los-rate)
    let derivative-term Kd * (los-rate - last-los-rate)
    set last-los-rate los-rate
    lt p-nav-gain * los-rate + integral-term - derivative-term
  ]

  ; --- Speed control ---
  fd speed
  ;if speed < (AAM-speed * unit-speed-factor) [set speed (speed + 0.01)]  ; Basic
  if time-burned < AAM-burn-time ; Speed calculated based on thrust, mass and burn time
  [
    ;show (word "setting speed to (" speed " * " patch-scale ") + " AAM-thrust " / " AAM-mass " * " 1 ") / " patch-scale " = " precision (((speed * patch-scale) + AAM-thrust / AAM-mass * 1) / patch-scale) 2)
    set speed (((speed * patch-scale) + AAM-thrust / AAM-mass * 1) / patch-scale) ; Multiply speed by scale, then bring the whole thing back down to patch scale, 1 is timestep (1 sec or tick)
    set time-burned time-burned + 1
  ]

  ; ----- ENDGAME -----
  ; Self-destruct at max time of flight
  if ticks > detonation-time
  [
    ask originator [set active-missiles (active-missiles - 1)] ; Not the best way to keep track
    set total-missed total-missed + 1
    die
  ]

  ; Impact target
  if distance target < 0.5
  [
    ask patch-here [set pcolor 9.9]
    ask target [die]
    set total-hit total-hit + 1
    ask originator [set active-missiles (active-missiles - 1)] ; Not the best way to keep track
    die
  ]
end

to operate-blue
  ask planes with [color = blue]
  [
    move-blue
    operate-radar
    fire-control
  ]
  ask interceptors with [color = blue]
  [
    fly-interceptors
  ]
end

to color-patches
  ; Visualize missile impacts
  ask patches with [pcolor > 0.001]
  [
    set pcolor pcolor * 0.99
    ask neighbors4 [set pcolor ([pcolor] of myself) * 0.95]
    if pcolor < 0.001 [set pcolor 0]
  ]
end

; -----------------------------

to go
  operate-red
  operate-blue
  color-patches
  tick
end
@#$#@#$#@
GRAPHICS-WINDOW
250
10
902
663
-1
-1
6.38
1
10
1
1
1
0
1
1
1
-50
50
-50
50
1
1
1
ticks
30.0

BUTTON
10
10
73
43
NIL
setup
NIL
1
T
OBSERVER
NIL
R
NIL
NIL
1

BUTTON
76
10
139
43
go
go
T
1
T
OBSERVER
NIL
G
NIL
NIL
0

INPUTBOX
5
285
78
345
plane-speed
200.0
1
0
Number

INPUTBOX
81
285
149
345
AAM-speed
600.0
1
0
Number

INPUTBOX
5
222
75
282
starting-red
10.0
1
0
Number

INPUTBOX
78
222
150
282
starting-blue
1.0
1
0
Number

MONITOR
951
11
1049
56
NIL
total-missiles-fired
0
1
11

MONITOR
1004
59
1074
104
NIL
total-missed
0
1
11

MONITOR
951
59
1001
104
NIL
total-hit
0
1
11

SLIDER
6
502
164
535
max-AAM-turn-rate
max-AAM-turn-rate
1
50
30.0
1
1
deg/s
HORIZONTAL

SLIDER
6
466
184
499
max-off-boresight-angle
max-off-boresight-angle
1
360
45.0
1
1
deg
HORIZONTAL

INPUTBOX
5
348
68
408
p-nav-gain
1.0
1
0
Number

TEXTBOX
6
46
183
88
Map scale: 1 patch = 1000 meters\nMap size is 100x100 km\nPatch scale set here ->
11
0.0
1

CHOOSER
100
127
192
172
units
units
"metric" "imperial"
1

INPUTBOX
124
64
191
124
patch-scale
1000.0
1
0
Number

INPUTBOX
124
348
174
408
Kd
0.1
1
0
Number

INPUTBOX
71
348
121
408
Ki
0.1
1
0
Number

SWITCH
5
185
136
218
can-engage-AAM
can-engage-AAM
0
1
-1000

PLOT
913
279
1113
429
Current Target SNR
Time
SNR
0.0
10.0
0.0
0.01
true
false
"" ""
PENS
"default" 1.0 0 -16777216 true "" "plot ([current-snr] of turtle inspect-aircraft)"

SLIDER
7
538
127
571
AAM-burn-time
AAM-burn-time
0
20
5.0
1
1
sec
HORIZONTAL

SLIDER
130
538
242
571
AAM-thrust
AAM-thrust
1000
20000
3000.0
1000
1
N
HORIZONTAL

SLIDER
7
573
106
606
AAM-mass
AAM-mass
50
200
85.0
1
1
kg
HORIZONTAL

INPUTBOX
912
117
996
177
inspect-aircraft
10.0
1
0
Number

MONITOR
912
181
1006
226
Airspeed (knots)
([speed] of turtle inspect-aircraft) * 1000 * 1.944
1
1
11

MONITOR
913
230
992
275
Heading (deg)
[heading] of turtle inspect-aircraft
2
1
11

TEXTBOX
179
358
238
400
Tuning for missile PID controller
11
0.0
1

TEXTBOX
156
231
230
277
Starting platform count for each side
11
0.0
1

SLIDER
101
429
195
462
max-tof
max-tof
5
150
50.0
5
1
sec
HORIZONTAL

SLIDER
6
429
98
462
R-max
R-max
1
20
5.0
1
1
km
HORIZONTAL

TEXTBOX
16
413
196
441
----- Other Missile Parameters -----
11
0.0
1

TEXTBOX
143
194
293
212
Allow launches
11
0.0
1

TEXTBOX
1018
249
1144
277
Data from aircraft's onboard radar
11
0.0
1

@#$#@#$#@
## WHAT IS IT?

This air-to-air combat model provides a simple two-dimensional representation of red and blue aircraft flying within a wrapped 100 km^2 area (each patch represents 1000 meters). The model's main focus is to provide an implementation of proportional navigation guidance law in the air-to-air interceptor missiles carried by the blue aircraft. The law is simplified to operate in the 2D environment, but still follows the same principle. The default settings are representative of short range missiles guided by an infrared seeker, such as older versions of the AIM-9 Sidewinder. The model has been extended so that the blue aircraft also carry an onboard radar. The radar has very limited range and does not feed other systems. It is based on characteristics of the AN/APQ-153 used in the F-5.

## HOW IT WORKS

The simulation will start by creating a number of red and blue aircraft in random locations on the map. The number of aircraft is user defined. Each blue aircraft will be assigned a target, which is the closest red aircraft to it at the time of assignment. The blue aircraft will pursue its target, and when in range it will engage by firing a missile. If the missile misses the target and terminates, the blue aircraft will attempt to re-engage if within launch parameters. If the missile impacts the target and destroys it, the blue aircraft will be assigned the next closest red aircraft as its new target. This will continue until there are no red aircraft remaining.

The main purpose of this model is to explore the [proportional navigation guidance law](https://en.wikipedia.org/wiki/Proportional_navigation) and the trajectory an interceptor missile will follow in order to impact its target. Proportional navigation's goal is to maintain the line-of-sight (LOS) rate from the missile to its target. In this model, the LOS rate is calculated by finding the difference between the missile's current relative bearing and the last recorded relative bearing in the previous timestep. The LOS rate is then fed into a PID controller as the process variable, with the setpoint being zero. The output value is used to control the intensity of the missile's maneuvering in order to maintain the LOS rate as close to zero as possible.

The onboard radar is modeled using a basic implementation of the radar range equation. Fixed parameters are entered based on the radar being modeled, such as transmitted power, antenna gain, bandwidth, and noise. Each aircraft is given a unique radar cross section (RCS) value. Aircraft within a pre-defined cone are considered to be in the radar's sensor volume, and calculations are performed with the equation to generate a signal to noise ratio (SNR) at each timestep. Future work will include a detection threshold, atmospheric noise, and an application of the radar's data to feed other targeting systems.

## HOW TO USE IT

Before running the simulation, a number of parameters can be set to vary aircraft speeds, the number of aircraft, and characteristics of the missiles. 

- patch-scale determines the factor used to scale other simulation values. This should not be changed.
- The units dropdown allows the user to enter either metric (m/s) or imperial (knots) values for the speed inputs.
- can-engage-AAM will prohibit blue from firing if set to OFF.
- The starting-red/starting-blue inputs allow the user to choose how many aircraft should be created.
- plane-speed and AAM-speed determine the constant speed of aircraft and missiles. Currently the AAM-speed value is not used.
- p-nav-gain, ki, and kd are parameters for tuning the PID controller on the missiles. These may cause undesired behavior if changed.

--- Other missiles parameters ---

- R-max is the maximum launch range for the interceptor missiles (km).
- max-tof is the maximum time of flight before missiles will terminate if they do not hit their target (sec).
- max-off-boresight-angle is the maximum angle away from an aircraft's nose that a missile can be launched. This is also used for radar azimuth scan limits (deg).
- max-AAM-turn-rate is that fastest possible rate that a missile can maneuver (deg/s).
- AAM-burn-time is how long the missile's engine will burn to produce thrust (sec).
- AAM-thrust is how much thrust the missile's engine will produce while it is burning (N).
- AAM-mass is the total mass of the missile (kg).

Once settings are configured as desired, the setup button should be pressed to initialize all the aircraft in the simulation. The go button should be pressed to begin the simulation. The inspect-aircraft input can be used to view data on a specific aircraft (ID). This is meant for blue aircraft. 


@#$#@#$#@
default
true
0
Polygon -7500403 true true 150 5 40 250 150 205 260 250

airplane
true
0
Polygon -7500403 true true 150 0 135 15 120 60 120 105 15 165 15 195 120 180 135 240 105 270 120 285 150 270 180 285 210 270 165 240 180 180 285 195 285 165 180 105 180 60 165 15

arrow
true
0
Polygon -7500403 true true 150 0 0 150 105 150 105 293 195 293 195 150 300 150

box
false
0
Polygon -7500403 true true 150 285 285 225 285 75 150 135
Polygon -7500403 true true 150 135 15 75 150 15 285 75
Polygon -7500403 true true 15 75 15 225 150 285 150 135
Line -16777216 false 150 285 150 135
Line -16777216 false 150 135 15 75
Line -16777216 false 150 135 285 75

bug
true
0
Circle -7500403 true true 96 182 108
Circle -7500403 true true 110 127 80
Circle -7500403 true true 110 75 80
Line -7500403 true 150 100 80 30
Line -7500403 true 150 100 220 30

butterfly
true
0
Polygon -7500403 true true 150 165 209 199 225 225 225 255 195 270 165 255 150 240
Polygon -7500403 true true 150 165 89 198 75 225 75 255 105 270 135 255 150 240
Polygon -7500403 true true 139 148 100 105 55 90 25 90 10 105 10 135 25 180 40 195 85 194 139 163
Polygon -7500403 true true 162 150 200 105 245 90 275 90 290 105 290 135 275 180 260 195 215 195 162 165
Polygon -16777216 true false 150 255 135 225 120 150 135 120 150 105 165 120 180 150 165 225
Circle -16777216 true false 135 90 30
Line -16777216 false 150 105 195 60
Line -16777216 false 150 105 105 60

car
false
0
Polygon -7500403 true true 300 180 279 164 261 144 240 135 226 132 213 106 203 84 185 63 159 50 135 50 75 60 0 150 0 165 0 225 300 225 300 180
Circle -16777216 true false 180 180 90
Circle -16777216 true false 30 180 90
Polygon -16777216 true false 162 80 132 78 134 135 209 135 194 105 189 96 180 89
Circle -7500403 true true 47 195 58
Circle -7500403 true true 195 195 58

circle
false
0
Circle -7500403 true true 0 0 300

circle 2
false
0
Circle -7500403 true true 0 0 300
Circle -16777216 true false 30 30 240

cow
false
0
Polygon -7500403 true true 200 193 197 249 179 249 177 196 166 187 140 189 93 191 78 179 72 211 49 209 48 181 37 149 25 120 25 89 45 72 103 84 179 75 198 76 252 64 272 81 293 103 285 121 255 121 242 118 224 167
Polygon -7500403 true true 73 210 86 251 62 249 48 208
Polygon -7500403 true true 25 114 16 195 9 204 23 213 25 200 39 123

cylinder
false
0
Circle -7500403 true true 0 0 300

dot
false
0
Circle -7500403 true true 90 90 120

face happy
false
0
Circle -7500403 true true 8 8 285
Circle -16777216 true false 60 75 60
Circle -16777216 true false 180 75 60
Polygon -16777216 true false 150 255 90 239 62 213 47 191 67 179 90 203 109 218 150 225 192 218 210 203 227 181 251 194 236 217 212 240

face neutral
false
0
Circle -7500403 true true 8 7 285
Circle -16777216 true false 60 75 60
Circle -16777216 true false 180 75 60
Rectangle -16777216 true false 60 195 240 225

face sad
false
0
Circle -7500403 true true 8 8 285
Circle -16777216 true false 60 75 60
Circle -16777216 true false 180 75 60
Polygon -16777216 true false 150 168 90 184 62 210 47 232 67 244 90 220 109 205 150 198 192 205 210 220 227 242 251 229 236 206 212 183

fish
false
0
Polygon -1 true false 44 131 21 87 15 86 0 120 15 150 0 180 13 214 20 212 45 166
Polygon -1 true false 135 195 119 235 95 218 76 210 46 204 60 165
Polygon -1 true false 75 45 83 77 71 103 86 114 166 78 135 60
Polygon -7500403 true true 30 136 151 77 226 81 280 119 292 146 292 160 287 170 270 195 195 210 151 212 30 166
Circle -16777216 true false 215 106 30

flag
false
0
Rectangle -7500403 true true 60 15 75 300
Polygon -7500403 true true 90 150 270 90 90 30
Line -7500403 true 75 135 90 135
Line -7500403 true 75 45 90 45

flower
false
0
Polygon -10899396 true false 135 120 165 165 180 210 180 240 150 300 165 300 195 240 195 195 165 135
Circle -7500403 true true 85 132 38
Circle -7500403 true true 130 147 38
Circle -7500403 true true 192 85 38
Circle -7500403 true true 85 40 38
Circle -7500403 true true 177 40 38
Circle -7500403 true true 177 132 38
Circle -7500403 true true 70 85 38
Circle -7500403 true true 130 25 38
Circle -7500403 true true 96 51 108
Circle -16777216 true false 113 68 74
Polygon -10899396 true false 189 233 219 188 249 173 279 188 234 218
Polygon -10899396 true false 180 255 150 210 105 210 75 240 135 240

house
false
0
Rectangle -7500403 true true 45 120 255 285
Rectangle -16777216 true false 120 210 180 285
Polygon -7500403 true true 15 120 150 15 285 120
Line -16777216 false 30 120 270 120

leaf
false
0
Polygon -7500403 true true 150 210 135 195 120 210 60 210 30 195 60 180 60 165 15 135 30 120 15 105 40 104 45 90 60 90 90 105 105 120 120 120 105 60 120 60 135 30 150 15 165 30 180 60 195 60 180 120 195 120 210 105 240 90 255 90 263 104 285 105 270 120 285 135 240 165 240 180 270 195 240 210 180 210 165 195
Polygon -7500403 true true 135 195 135 240 120 255 105 255 105 285 135 285 165 240 165 195

line
true
0
Line -7500403 true 150 0 150 300

line half
true
0
Line -7500403 true 150 0 150 150

pentagon
false
0
Polygon -7500403 true true 150 15 15 120 60 285 240 285 285 120

person
false
0
Circle -7500403 true true 110 5 80
Polygon -7500403 true true 105 90 120 195 90 285 105 300 135 300 150 225 165 300 195 300 210 285 180 195 195 90
Rectangle -7500403 true true 127 79 172 94
Polygon -7500403 true true 195 90 240 150 225 180 165 105
Polygon -7500403 true true 105 90 60 150 75 180 135 105

plant
false
0
Rectangle -7500403 true true 135 90 165 300
Polygon -7500403 true true 135 255 90 210 45 195 75 255 135 285
Polygon -7500403 true true 165 255 210 210 255 195 225 255 165 285
Polygon -7500403 true true 135 180 90 135 45 120 75 180 135 210
Polygon -7500403 true true 165 180 165 210 225 180 255 120 210 135
Polygon -7500403 true true 135 105 90 60 45 45 75 105 135 135
Polygon -7500403 true true 165 105 165 135 225 105 255 45 210 60
Polygon -7500403 true true 135 90 120 45 150 15 180 45 165 90

sheep
false
15
Circle -1 true true 203 65 88
Circle -1 true true 70 65 162
Circle -1 true true 150 105 120
Polygon -7500403 true false 218 120 240 165 255 165 278 120
Circle -7500403 true false 214 72 67
Rectangle -1 true true 164 223 179 298
Polygon -1 true true 45 285 30 285 30 240 15 195 45 210
Circle -1 true true 3 83 150
Rectangle -1 true true 65 221 80 296
Polygon -1 true true 195 285 210 285 210 240 240 210 195 210
Polygon -7500403 true false 276 85 285 105 302 99 294 83
Polygon -7500403 true false 219 85 210 105 193 99 201 83

square
false
0
Rectangle -7500403 true true 30 30 270 270

square 2
false
0
Rectangle -7500403 true true 30 30 270 270
Rectangle -16777216 true false 60 60 240 240

star
false
0
Polygon -7500403 true true 151 1 185 108 298 108 207 175 242 282 151 216 59 282 94 175 3 108 116 108

target
false
0
Circle -7500403 true true 0 0 300
Circle -16777216 true false 30 30 240
Circle -7500403 true true 60 60 180
Circle -16777216 true false 90 90 120
Circle -7500403 true true 120 120 60

tree
false
0
Circle -7500403 true true 118 3 94
Rectangle -6459832 true false 120 195 180 300
Circle -7500403 true true 65 21 108
Circle -7500403 true true 116 41 127
Circle -7500403 true true 45 90 120
Circle -7500403 true true 104 74 152

triangle
false
0
Polygon -7500403 true true 150 30 15 255 285 255

triangle 2
false
0
Polygon -7500403 true true 150 30 15 255 285 255
Polygon -16777216 true false 151 99 225 223 75 224

truck
false
0
Rectangle -7500403 true true 4 45 195 187
Polygon -7500403 true true 296 193 296 150 259 134 244 104 208 104 207 194
Rectangle -1 true false 195 60 195 105
Polygon -16777216 true false 238 112 252 141 219 141 218 112
Circle -16777216 true false 234 174 42
Rectangle -7500403 true true 181 185 214 194
Circle -16777216 true false 144 174 42
Circle -16777216 true false 24 174 42
Circle -7500403 false true 24 174 42
Circle -7500403 false true 144 174 42
Circle -7500403 false true 234 174 42

turtle
true
0
Polygon -10899396 true false 215 204 240 233 246 254 228 266 215 252 193 210
Polygon -10899396 true false 195 90 225 75 245 75 260 89 269 108 261 124 240 105 225 105 210 105
Polygon -10899396 true false 105 90 75 75 55 75 40 89 31 108 39 124 60 105 75 105 90 105
Polygon -10899396 true false 132 85 134 64 107 51 108 17 150 2 192 18 192 52 169 65 172 87
Polygon -10899396 true false 85 204 60 233 54 254 72 266 85 252 107 210
Polygon -7500403 true true 119 75 179 75 209 101 224 135 220 225 175 261 128 261 81 224 74 135 88 99

wheel
false
0
Circle -7500403 true true 3 3 294
Circle -16777216 true false 30 30 240
Line -7500403 true 150 285 150 15
Line -7500403 true 15 150 285 150
Circle -7500403 true true 120 120 60
Line -7500403 true 216 40 79 269
Line -7500403 true 40 84 269 221
Line -7500403 true 40 216 269 79
Line -7500403 true 84 40 221 269

wolf
false
0
Polygon -16777216 true false 253 133 245 131 245 133
Polygon -7500403 true true 2 194 13 197 30 191 38 193 38 205 20 226 20 257 27 265 38 266 40 260 31 253 31 230 60 206 68 198 75 209 66 228 65 243 82 261 84 268 100 267 103 261 77 239 79 231 100 207 98 196 119 201 143 202 160 195 166 210 172 213 173 238 167 251 160 248 154 265 169 264 178 247 186 240 198 260 200 271 217 271 219 262 207 258 195 230 192 198 210 184 227 164 242 144 259 145 284 151 277 141 293 140 299 134 297 127 273 119 270 105
Polygon -7500403 true true -1 195 14 180 36 166 40 153 53 140 82 131 134 133 159 126 188 115 227 108 236 102 238 98 268 86 269 92 281 87 269 103 269 113

x
false
0
Polygon -7500403 true true 270 75 225 30 30 225 75 270
Polygon -7500403 true true 30 75 75 30 270 225 225 270
@#$#@#$#@
NetLogo 6.3.0
@#$#@#$#@
@#$#@#$#@
@#$#@#$#@
@#$#@#$#@
@#$#@#$#@
default
0.0
-0.2 0 0.0 1.0
0.0 1 1.0 0.0
0.2 0 0.0 1.0
link direction
true
0
Line -7500403 true 150 150 90 180
Line -7500403 true 150 150 210 180
@#$#@#$#@
0
@#$#@#$#@
