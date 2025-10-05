# Robot Design Evolution — TEAM DELTA (WRO 2025)

This directory presents the **evolution of TEAM DELTA’s robot design** through several key development stages, from the first prototype to the final competition model **RSLN118**.  
Each version introduces mechanical, structural, and electronic improvements toward achieving a compact, modular, and high-performance autonomous vehicle.

---

## Version V1
The **initial concept prototype**, designed to validate the overall geometry and mechanical feasibility of the drivetrain and steering system.  
This version consists of a **flat, single-level chassis** with basic mounting points for the DC motor and steering servo.

![Version V1](v1.jpg)

---

## Version V2
Added **partial enclosures** and a more rigid front structure to reinforce the servo and motor mounting.  
Wheel positioning and clearance were refined, and the first **servo linkage** adjustments were introduced.

![Version V2](v2.jpg)

---

## Version V3
Introduced a **more robust chassis** with a thicker mid-frame and clearer motor alignment.  
The rear axle and pulley system were optimized for better torque transfer.  
Color-coded components were implemented (red for drivetrain elements) to improve part identification during assembly.

![Version V3](v3.jpg)

---

## Version V4
A **fully functional mechanical prototype**, featuring precise Ackermann steering geometry and improved structural rigidity.  
This version introduced **rugged wheels**, refined servo mounting, and the first **differential system** for rear traction balance.

![Version V4](v4.jpg)

---

### Note
The differences between **V4**, **V5**, and **V6** were **minor and functionally negligible**.  
These intermediate versions mainly involved small **dimensional corrections**, **cable routing optimization**, and **mounting hole alignment**.  
For simplicity and traceability, only **V4 and V6.5** are presented here, as they represent the most significant design milestones.

---

## Version V6.5
Introduced a **multi-floor modular structure**, separating mechanical, electrical, and computational layers.  
The design now accommodates the **Raspberry Pi 5**, **ESP32**, **motor drivers (DRV8871)**, and **sensors** within dedicated compartments.  
Improved frame reinforcements and integrated brackets provide cleaner wiring paths and enhanced serviceability.

![Version V6.5](v6.5.jpg)

---

## Final Version — RSLN118
The **official competition model** for the **WRO 2025 – Future Engineers** category.  
This version integrates all electronic and mechanical systems into a refined **three-floor chassis**:

- **Lower Floor:** drivetrain, motor drivers, and servo system.  
- **Middle Floor:** ESP32, power regulators, and wiring management.  
- **Upper Floor:** Raspberry Pi 5 and Logitech C920 camera mount.

The RSLN118 represents the culmination of all previous iterations — optimized for **stability, modularity, and performance** in real-world conditions.

![RSLN118](RSLN118.jpg)

---
From **V1 to RSLN118**, each iteration brought the robot closer to a complete mechatronic system.  
The evolution reflects continuous improvements in **mechanical strength, electronic integration, and design modularity**, resulting in a robust and competition-ready autonomous platform.
