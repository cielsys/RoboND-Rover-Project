# Udacity RoboND Project: Rover Search and Sample Return
#### SubmissionNotes -ChrisL
Originally Submitted/Reviewed July 10/11, 2018 as master:be752dbdf8e3b1d435221dae5435319cb09449ad
Now containing post-submission notes regarding further, independent development

[//]: # (Image References)
[image1]: ./misc/rover_image.jpg
[image2]: ./calibration_images/example_grid1.jpg
[image3]: ./calibration_images/example_rock1.jpg

****
## Important Notes for review:
  My files are named differently and in a different location than the original project.

* My testing notebook and python files and  are located in $REPO/cl_code/
* Main file is rovermain.py in that directory
* The original code has not been modified

  This is a fun project! The Jupyter notebook exercises were manageable but required thought and understanding.
The actual simulator control was somewhat involved. And trying to graft things back and forth between the notebook, which is good for debugging,
and the actual sim control application was tricky. I found that I wanted to refactor things so that they would be using almost entirely the same code and in retrospect I should have just worked to get the assingment done on time.

The Sim controller actually does usually map more than 50% of the terrain with 70% fidelity.
### ======= Refactoring ==============
I am performing a major refactor of the code for several reasons:
* To consolidate the Notebook and SimControl code
This is to allow easy switching between Notebook dev/debug and SimControl testing.<br />
In particular I would like to be able to selectively redirect telemetry and image input between the the Sim and pre-recorded telemetry and have it pass through identical processing code.
Furthermore I have a longer objective to use this code with an actual Rover I have been working on as a side project, 
while maintaining compatibility with the RoverSim.
* To generally convert to best coding practices such as:
    * Abundantly clear, unambiguous, 'normalized' identifier names, ie variables, functions, etc<br />
Part of this renaming is to experiment with compromises between python PEP conventions and my own personal
guidelines borne of experiences in other languages, eg C,C++ and C# so as to develop a personal style that fits
comfortably with both conventions. See 'Coding Guidelines' below for discussion
    * To eliminate global variables, as much as possible, or at least to prefix them with `g_`
    * To separate related code into independent, reusable modules

### ======= Empirical Calibration ===
#### --- Rover POV camera calibration
#### --- Rover dimensions and coordinate system relationships

### ======= DevEnvNotes ==============
Notes to myself regarding the development environment
1. Enable python env:<br />
`source activate RoboND` #Linux<br />
`activate RoboND` #Win<br />

2. Run Rover program:<br />
`python RoverMain.py` # from shell<br />
`exec(open("RoverMain.py").read())` # from interpreter or notebook<br />
`exec(open("./RoverMain.py").read(), globals())`<br />
Start simulator, select Autonomous. Must happen after RoverMain start. Sometimes required twice.

3. Killing the program in windows:<br />
Ctrl-Break will effectively break the async stuff and end the python session.
Ctrl-c enter repeat will sometimes work

### ======= RoverSim Notes ==============
#### ---Keyboard shortcuts
* F1 Reset scene: Reset Manual/Autonomous mode
* F12 Quit
* wasd Steering
* Tab Toggle cam view
* r Toggle recording
* g toggle grid display
* c Toggle Rover POV inset

#### ---Mouse shortcuts
* MouseMove Steering
* MouseScroll Zoom
* MouseButMid Reset zoom
* MouseButRight+Mouse Pan/Tilt
* MouseButLeft Pickup nugget

### ---Rover Behaviour:
* V max ~=5m/s
* A = 1m/s/s (only throttle position)
* Brake power is much stronger than throttle
* -A ~=2.5m/s
* Slope affects speed
* Drag appears to be proportional to V**2
* Turning increases drag alot
* Steering is active at 0V

### --- Rover Environment:
* Tracks disappear after a minute or so
* Some bajadas are very sand colored
* IsNear is a basketball foul box shape about 1m long and outer wheel edge wide
* You can get high sided on gold nuggets

### Coding Guidlines

### Links
[RoverReviewRubric](https://review.udacity.com/#!/rubrics/916/view)<br />
[MarkDownCheatsheet0](https://github.com/adam-p/markdown-here/wiki/Markdown-Cheatsheet "MDCheat0")<br />
[MarkDownCheatsheet1](https://markdown-guide.readthedocs.io/en/latest/basics.html "MDCheat1")<br />

