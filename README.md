# Adaptive Active Disturbance Rejection Control

## Introduction
<p style="text-align: justify">
The altitude control of a quadrotor unmanned aerial vehicle is treated using its altitude dynamics in hover mode. An LMS-based Adaptive Active disturbance rejection control (AADRC) is proposed to compensate for disturbance effects resulting in smoother control.</p>

## Description
An LMS algorithm for the adaptation of the observer gain was presented to automatically tune the ESO observer. The ESO observer linearizes the altitude dynamics. Subsequently, a PD controller was implemented for the linearized dynamics and was successfully simulated in Matlab

The Altitude dynamics of a Quadrotor is given as follows:  
![](formula0.png)  
  

Formulation of the ADRC Controller:  
![](formula4.png)    
    
Formulation of the AADRC Controller using LMS:  
![](formula1.png)  
![](formula2.png)  
![](formula3.png)  

## Results
The figure below shows the result of applying the AADRC for Altitude control
![](results_plots.png)  
