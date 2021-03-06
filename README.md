# Joint-optimization-of-minimum-energy-path-trajectory-while-battery-swapping-of-UAV-in-3D-aerial-High
-Deepali Kumari, Electrical Engineering, B. Tech, B.I.T. Sindri
## Abstract-the H-UAV (Helper Unmanned Aerial Vehicle) trajectory to minimize the total energy consumption while completing its mission of battery swapping of UAV (Unmanned Aerial Vehicle) in 3D aerial Highways. Formulation is projected in three phases - I Phase: TF (Target Flight), II phase: SF (Swapping Flight) and III Phase: RF (Return Flight). Within energy budget and minimum time, the optimization is laid out for minimum path trajectory during the flight. The study is carried out targeting a single UAV in 3D highways.

## I. System Model
### A. UAV
The initial location of the UAV at time (to), at which it sends the signal to the ground station, is represented by π€0=[π₯π€0,π¦π€0,π§π€0]. Itβs flying speed is π£0=[π£0(π₯),π£0(π¦),π¦0(π§)]. te represents the exhaustion time- the time when the UAV would be completely battery dead. we denote the coordinates of UAV at the exhaustion time, π€π=[π₯π€π,π¦π€π,π§π€π]. Let wi = [π₯(π‘π),π¦(π‘π),π§(π‘π)] be the coordinates of UAV along its direction (πβ ) towards the destination where wi = {w1, w2 ,β¦,wk}and ti = {t1,t2,β¦.tk} for i={1,2β¦..,k}.     

### B. H-UAV
Vmax, Ho, and ts represent the maximum velocity, initial coordinates and switching/swapping time respectively. Ho would be considered as the frame of reference, π»0=[π₯β0,π¦β0,π§β0]. H-UAV is docked on the top of the building to ensure the minimum consumption shown in the figure1.
![battery swapping](https://user-images.githubusercontent.com/87405534/126459538-217549ab-96c8-48e4-b518-0e07042ef48a.jpg)

### C. Trajectory Design Model          
Targeting the UAV in 3D aerial Highways is critical and maintaining a parallel precision of H-UAV with UAV is highly demanding. The model proposed is stimulated in assumption of a linear trajectory of UAV. The UAVβs waypoints or nodes (wi) are estimated on the itβs direction vector (πβ ). In reference to initial location of H-UAV the paths are plotted for the trajectory as depicted in the figure2.

![Untitled Diagram2](https://user-images.githubusercontent.com/87405534/126459789-ef93b1bc-c35b-4b6e-8576-2b250b8c5960.jpg)


wj = (wi - h(z)) + (vo. ts) πβ , is set of coordinates of H-UAV along the direction of UAV, after swapping the battery, where j={1,2β¦..,k} and h(z) is the height difference between UAV and H-UAV in 3D Highways while swapping that need to be maintained.

#### Explanation:
wo and t0 is the initial position and time of UAV, when it sent the signal to the ground station about its battery deficiency, respectively. Objective is to design the minimum time trajectory for the H-UAV to reach the concerned waypoint, determined by Vmax. So, in I Phase-TF (Target Flight), waypoints satisfying t(oi(min))=ti are the concerned waypoints for trajectory optimization because if the H-UAV doesnβt make to UAVβs waypoint at the time equal to ti then it wonβt be able to catch the UAV(because UAV will move forward).     
Assume:   
t(o1(min)) >t1    
t(o2(min)) >t2     
t(o3(min)) >t3    
t(o4(min))=t4,     
t4 onwards the waypoints are the concerned waypoints for calculating the energy.     
So, finally the concerned waypoints are:     
W = {w4, w5, w6, w7,β¦..}
The other assumption made in due process is that H-UAV is not hovering in the space, implying that the H-UAV has to reach the waypoint(node) in the same time, the UAV takes to reach the node. Addressing constraint in II phase-SF (Swapping Flight) ti + ts β€ te is beneficial, so that the flying and switching time is less than or equal to exhaustion time. So, assume w8, w9, onwards doesnβt satisfy the constraints and eliminated from the concerned nodes. Hence, Minimum energy is estimated on W={w4, w5, w6, w7} and the viable node is set for the mission.         

### D. Energy Model
The energy consumption of the H-UAV consists of two types: Propulsion(flying) energy consumption and Swapping Energy. The former measures the energy consumed to fly to the target. The later is the mechanical energy consumed by the parts deployed to swap the battery while performing the parallel operation. The power consumption of a rotary-wing UAV flying at velocity is given as:

![Capture](https://user-images.githubusercontent.com/87405534/126458248-1deeded5-fd73-4a13-ac2b-0edaa37bdeac.PNG)
![Capture2](https://user-images.githubusercontent.com/87405534/126460128-54268a26-73bc-414b-b3d2-2d91c036e83c.PNG)

Blade profile power, parasite power, and induced power are needed to overcome the profile drag of the blades, the fuselage drag, the induced drag of the blades, respectively.  
The flying energy of H-UAV from Ho to wi is given by:   
Efly(oi)=Pfly(Vmax) . t(oi)                                                    
The flying energy of H-UAV from wi to wj is given by:    
Efly(ij)= Pfly(Vo) . ts                                                        
This is the fixed energy.    
The mechanical swapping energy of H-UAV from wi to wj is assumed to be constant (Eswap).      
Eswap= Pswap(Vo) . ts                                                         
The flying energy of H-UAV from wj to Ho is given by:
Efly(jo)=Pfly(Vmax) . t(jo)                                                   
So, the total energy consumption by the H-UAV during its journey is expressed as:    
Etotal(consumed) = Efly(oi)+ Efly(ij)+ Eswap + Efly(jo)                      

## II. Energy-Efficient H-UAV with Path & Time optimization  

Minimum energy consumption by H-UAV is determined by optimizing the mission time of H-UAV. We have assumed that the H-UAV flies at vmax from Ho to wi and while returning from wj to Ho it flies back to the port at the speed of Vmax.   
The formulation of the problem with certain constraints are:   
![Captureguyfh](https://user-images.githubusercontent.com/87405534/126461503-1258f5e4-9c99-4602-95b5-5ca658887bba.PNG)

C1: The time stipulated for the UAV to reach a particular node should be the same as the time taken by H-UAV to fly over the targeted node, inferring minimum time loss at the node, since hovering energy is assumed to be negligible.                    
C2: The time is taken by phase I: Target flight and phase II: Swapping flight should be less than exhaustion time of UAV, else the UAV would be dead before swapping.             
C3: The total energy consumed in all 3 phases should be less than or equal to the residual energy of H-UAV.        

## III. Limitations
This research makes the first attempt to design the dynamic model of battery swapping of UAV in 3-D aerial Highway itself. The proposed approach is modeled with assumptions of no hovering energy in the II phase; swapping flight and linear part of the UAV in its trajectory. In a practical scenario, the pathways would be more complex and in such a case, the assumptions would be denied. Another important speculation is this research is devoted to a serve single UAV via H-UAV.

## IV. Future Research
The model can be intended for multiple UAVs, thatβs swarms of UAVs, in 3-D aerial Highways. It can be made more dynamic when the fleet of H-UAVs co-ordinate their work in aerial Highways with swarms of UAVs. Another way around to further extend the experimentation is when a single H-UAV, serves multiple UAVs in a restricted area in one round journey.

