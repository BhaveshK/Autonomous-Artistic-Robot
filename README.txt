2012- CS308 Group 18 : Autonomous artistic robot
================================================

Group Info:
------------
+ Vineet Kakati 
+ Ganesh Dhuri
+ Bhavesh Khetpal


Project Description
-------------------
The project comprises of the creation of  a swarm  of robots capable of drawing a multicoloured image autonomously. This image can be any complex figure, but to start initially we will be providing simple shapes and objects. This multicoloured image will have the three basic colour components, RGB (Red, Green & Blue).
The entire module consists of a main computer system which will process the required image using Matlab software. The Matlab has inbuilt features to return the pixel matrix of any given input image. It can also break the pixel matrix into its constituent matrices on the basis of the colour, i.e. each different colour amongst RGB will have a different pixel matrix. After further vectorization, the main computer will give the inputs to the bots via Zigbee protocol. Each bot is responsible for drawing the only part of the image that is coloured in the colour it is responsible for (i.e. any one out of RGB).
The robots are equipped with the drawing utility, i.e. a servo motor that is capable of basic up and down motion. This motor controls the movements of a pen/marker/paint etc , i.e. whatever we use as a drawing tool.


Technologies Used
-------------------

+ Embedded C
+ MATLAB
+ Xbee
+ Servo motor   


Instructions
=========================
Instructions to run the project:
1.Create a bitmap image with Paint with the desired shapes in colours Red,Blue and Green   say rgbim.bmp" and save it in Matlab home directory.

2.Create 3 new projects in AVR Studio named rfinal,bfinal and gfinal.Add the files   ‘rfinal.c’,'bfinal.c' and 'gfinal.c'as source files and compile the code.

3.Burn the generated files ‘rfinal.hex’,'bfinal.hex' and 'gfinal.hex' to the respective   bots.

4.Run the command ‘execfinal(im);’ in the Matlab console.This file calculates all the   required lengths, angles etc.(with the help of other matlab source files) and sends   the required commands wirelessly via Xbee transmission port.

5.Place the bots in a spacious location. The Robots traverse along the shapes depicted in   the image as required!



References
===========
+ www.nex-robotics.com/spark-v-robot.html

+ www.e-yantra.org

+ Paintwithrobots.fr.mu , http://paintwithrobots.fr.mu/THE ROBOTPAINTER SCARABOT

+ A thinning algorithm by contour generation, Paul. C. K. Kwok, Communications of the        ACM, November 1988 .

+ Raster-to-vector Conversion by Line Fitting Based on Contours and Skeletons, Osamu Hori   and SatohideTanigawa, Research and Development Center, Toshiba Corporation1,   KomukaiToshibic-cho, Saiwai-ku, Kawasaki 210, JAPAN .

+ SPARK V ATMEGA16 Robot Research Platform Hardware and Software Manual.



Youtube link of demo
=====================
http://www.youtube.com/watch?v=qYQ4LRSmDJw

