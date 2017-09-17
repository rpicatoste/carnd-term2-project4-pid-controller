# Self-Driving Car Nanodegree Program - Term 2
## Project 4 - PID Control
**Ricardo Picatoste**

## Notes
The project has been compiled in Windows 10 using the "Bash on Ubuntu on Windows", generating the make file with "cmake CMakeLists.txt" and then running make. 

I have used a tab width of 4 spaces, taking care of the matrices alignment. I hope it's ok, I prefer it like that for programming.

## Results

The car does the whole track without leaving the road. The throttle has been fixed to a 50% percent higher values than the 0.3 original to make the car go faster, while keeping it on track.

The tuning has been manual. First the P and D terms, since those have the most obvious effect on the control. First obtaining only with P an oscillatory car that, other that the oscillation, stays around the target positions. Then with the D term, smooth it until the oscillations are mostly gone.

Finally I added a bit of I term, with the intention of compensating for systematic errors such like steering or sensor bias.

I implemented the PID in a different way from the lessons. The P and D parts are mostly the same, but the integral part includes anti-windup, in order to avoid controller windup due to the actuation saturation (the bounds given for the steering between 1 and -1). This anti-windup will check if there was saturation in the last calculated and applied control action, and if so, instead of keeping summing error in the accumulator, it will discharge the accumulated error a bit. This prevents big oscillations given when the controller reaches the mentioned saturation limits.
