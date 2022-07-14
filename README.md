 # Model Predictive Control
 #### Comparisons of LSQ DMC, QDMC, Constrained QDMC, Robust constrained QDMC and Neural MPC
 

*The inspiration for this repo originated from the course - Process Systems Modelling taught by Professor Fernando Lima at Carnegie Mellon University in the Spring of 2022 and later with the Non Linear Programming Course by Professor Lorenz T Biegler in Fall 2022. The idea grew outside the course, at which it was a much simplified version and I am thankful to my teachers for instilling within me their trust and guiding me in my endeavour even outside the classroom.*

The optimal control design of systems is an important problem of control theory. A powerful way of investigating this problem is to use model predictive controllers (MPCs) which are known to be popular in many fields of applications (Qin and Badgwell, 2003).

I try to investigate the control schematics of the Quadratic Tank Process which involves 4 interconnected water tanks. Each of these tanks has an orifice located in the bottom so that fluid may drain by way of gravity. These orifices are removable and constructed with various diameters allowing the system to have different sets of parameters. In this arrangement, the tanks are split into two vertical sets so that one tank is positioned over another allowing fluid to drain from one tank to the second and then finally to a basin for recycling. The fluid flow from two positive displacement pumps is split between an upper tank and the opposing set's bottom tank. The main focus of this system is to mandate the height of fluid in the bottom two tanks via a control system. This control system uses the tank's current fluid height as input to make proper height adjustments by varying the output of pump speed, which imparts an inlet flow to the tanks.

The study is of two inputs (pump flow rates) which can be manipulated to control the two outputs (Tank 1 and Tank 2 levels). The system exhibits interacting multivariable dynamics because each of the pumps affect both of the tank levels.
<p align="center">
 <img width="460" height="300" src="QuadTank.png">
</p>

### The Data modelling Algorithm works in the following way :
 <p align="center">
 <img width="550" height="150" src="MPC.png">
</p>

### 1 . Modeling and simulation of the ODE system
The system is designed to have two tank heights (h<sub>1</sub>, h<sub>2</sub>) as output, four tank heights (h<sub>1</sub>, h<sub>2</sub>,h<sub>3</sub>, h<sub>4</sub>) as state variables, and two pump volumetric flow rates (q<sub>a</sub>, q<sub>b</sub>) as inputs. The nonlinear ODE model of the four-tank system is obtained using mass balance equation and Bernoulli’s law.

### 2 . Representing the ODE system in the continuous linear state-space form
State-space form is a mathematical model of a physical system as a set of inputs, outputs and state variables related by differential equations. State variables are variables with values that change over time, depending on the values they have at given time and on the externally imposed values of input variables. Output variables values depend on the values of the state variables. The state-space representation can be expressed in continuous or discrete form which are discussed below and in the next section.

From the ODE system, it can be represented as a continuous linear state-space form:
<p align="center">
 x'=Ax'+Bu'  <br />
 y'=Cx'+Du' 
</p>

### 3 . Converting the ODE system to discrete-time state-space representation
The system is then converted to discrete-time state-space form as follows, in which φ and Γ are discrete linearization representations.
<p align="center">
x(k+1) = φx(k)+Γu(k) <br />
y(k) = Cx(k)+Du(k)
</p>

φ and Γ are calculated from A, B, I matrices and discrete time Δt using the numerical equations, while C and D remain unchanged. 

### 4 .  Creating the Step-response model
The finite step response model is obtained by making a unit step input change to a process operating at a steady-state. The steady state solution of the system is
found either by solving the ODE's simultaneously for points where derivatives are collectively zero or from experimental investigations which show points of stability which are usually the initial points.

### 5 .  Checking the system dynamics
To check whether the system is controllable in a non minimum phase environment by fixing h_3 and  h_4 the control (ctrl) package in python was used which required as its input the φ and Γ matrices and returned a full rank controllability matrix ensuring that the system is observable and controllable.
Minimum phase systems are those which have zero poles in the transfer function dynamics while finding the gain matrix. (Denominator zero) and are inherently very unstable.

### 6 .  Setting up the Controller

The value of different predictive output ŷ(k) is initially calculated by the equation: <br />

$$\left( ŷ(k)\right) = \left(\sum_{i=1}^{N-1} s_i Δu(k-i) +s_{N}u(k-N)\right)$$

This equation assumes that all step response coefficients which are larger than step N are equivalent to N, which means that the system already reaches steady state. Therefore, part of the equation is simplified to s<sub>N</sub> u(k-N). Besides, the difference between measured output and predictive output is named disturbance.

$$d(k)=y(k)-ŷ(k)  $$                                                        
                                                                
$$\left( ŷ^c (k+j)\right)=\left(\sum_{i=1}^{j} s_i Δu(k-i+j) +\sum_{i=j+1}^{N-1}s_i Δu(k-i+j) + s_N u(k-N+j)+d ̂(k+j)\right)$$

$$\left( Ŷ^c=S_f Δu_f+S_past Δu_past+s_N u_p+d \right)$$ ̂                                                                                                  

