# MotionCal's quality measurements

## The [original question](https://forum.pjrc.com/index.php?threads/motion-sensor-calibration-tool-parameter-understanding.59277/#post-227964)

```
I am using Motion Sensor Calibration Tool for calibrate the Magnetometer Sensor. I want to know about the parameters which are showing quality of calibration.

The parameters are:
1. Gaps
2. Variance
3. Wobble
4. Fit Error

What is significant of above parameters and how they are useful for quality of calibration? I want to know about those parameter because I am trying to make the firmware which is useful without Motion calibration Tool.

Please help me to understand this and Many many thanks in Advance.

Thanks and Regards,
Ankit
```

## Paul Stoffregen's [answer](https://forum.pjrc.com/index.php?threads/motion-sensor-calibration-tool-parameter-understanding.59277/#post-227990).

The most precise info about each of these is the MotionCal source code.

https://github.com/PaulStoffregen/MotionCal

Look at quality.c.

As you move the sensor, the 3D visualization should appear as a sphere.

To answer your question very briefly...

Gaps tries to measure how much of the sphere's surface is missing data points.

Variance tries to measure how much of the data is not located on the (imagined) surface of the sphere.

Wobble tries to show how far an estimated the "center of mass" is from the ideal center.

Fit error comes from the algorithm which actually produces the calibration coefficients. The main reason I wrote MotionCal is because the fit error can sometimes be low for a terrible set of data. Think of fitting a linear line onto a set of 2D data on a graph. You can have a group of incomplete data on just one tiny part of the graph, which by chance happen to fall well onto a straight line. Intuitively you wouldn't draw a line through such a small group and claim that line represents a trend across the large graph. A set of poor magnetic measurements which happen to fit the model well is the same sort of situation, but fitting a sphere onto 3D data, rather than fitting a linear line to 2D data.

Also, if you attempt calibration near the presence of a strong magnetic field or too close to iron or steel objects that run parallel to power wires or anything else that induces a magnetic field, you can end up with an ellipse or other weird shape. The fitting algorithm is remarkably good at transforming an obviously wrong ellipse into coefficients. Fit error alone can't warn you that you've got garbage data. The best way to tell if your calibration is based on good measurements is to visually confirm the 3D animation really is a sphere.

I designed MotioCal's data management to prioritize replacing bad data points with good ones, so the results (usually) converge rapidly to a sphere. But if you start out with really horrible data, well, I'm not that good at anticipating such scenarios. Some really bad initial data cases can cause it to never converge to a useful data set. You can experience this if you hold a small magnet in one hand while moving the sensor around with your other hand. It's kind of fun to wave the magnet near the sensor and see the calibration go wild. You can also see my attempts to make MotionCal converge back to sphere by purging the bad measurements over time, and the limitations it has. The important point to remember is MotionCal isn't treating data in a dispassionate or purely objective way. It's making prioritized decisions to try to keep good data and purge bad. But MotionCal is far from the sort guarantee of convergence you might expect from something designed by academians who specialize in rigorous mathematical proofs. Much of how MotionCal works is ad-hoc stuff I just made up. I'm pretty good, but not that good.

Those 3 other numerical estimates are a poor substitute for human vision and intelligent judgement. They mainly just provide a way to keep the button to commit the calibration disabled in the early phase while the data is so obviously not a good sphere. Don't get seduced into the idea that those 4 numerical estimates are really meaningful. The first 3 are just crude estimates I made up. They are based solely on my own personal intuition & experimentation. They're not any sort of well recognized algorithm, only things I made up for the sake of a user-friendly GUI. None of the 4 are perfect. The idea is that hopefully at least 1 of the 4 will keep the send cal button disabled while the data is so bad that it should not be used. Those numbers aren't meant to prove the data is good. They only reject very bad data, and even that isn't 100% guaranteed.

The real validation of quality is human judgement of whether the 3D animation actually resembles a perfect sphere. That's why I went to so much trouble to mess with OpenGL programming!