# Notes for Robot Assembly

These are some important things to keep in mind when assembling and powering the robot. One of the main challenges we faced was ensuring stable power delivery to the **Raspberry Pi 5** when running high-demand peripherals like the camera.

---

# Power Supply Issues and Solution

During testing, we experienced several power-related problems when trying to supply the **Raspberry Pi 5**:

1. **Using a Powerbank:**  
   - The Pi could boot normally.  
   - However, once the **camera code** started running, the board would suddenly **shut down**.  
   - This was due to the **current demand spikes** of the camera exceeding the powerbank’s stable output capacity.

2. **Using a LiPo Battery + DC-DC Regulators:**  
   - We tested with a **12V battery** connected to different **step-down DC-DC converters**.  
   - The Pi would **not even boot** in this setup.  
   - The reason was insufficient **current supply** during the boot process, as the converters could not provide enough amperage.

3. **Final Solution:**  
   - We solved the problem by combining:  
     - A **DROK Buck Converter** (12V to 5V, up to 5A output).  
     - A **Huawei powerbank** as a backup and stable source.  
   - This setup provided enough **voltage stability** and **current capacity** to power the Raspberry Pi 5 reliably, even when running heavy tasks like camera processing.

**Conclusion:**  
Stable and sufficient **current delivery** is critical for the Raspberry Pi 5, especially when peripherals like cameras are active. Using a high-quality **buck converter rated at ≥5A** combined with a reliable powerbank eliminated the shutdown and boot issues.
