# SedNav_Windsensor

An open-source wind sensor design, taking inspiration from the project [Windsensor Yachta](https://github.com/norbert-walter/Windsensor_Yachta) by Norbert Walter.

It uses the same firmware, modified to work with this design.

The aims of this derivation are :
- To simplify the building process, by making the PCB easier to solder by hand
- To simplify the flashing, by using the ESP32C3 or C6 with a built-in USB interface (no more need for a serial to USB converter)
- To reduce the size of the wind sensor, by shrinking the pcb and reworking the design, in order to reduce the wind resistance, making it more resilient to winds.
- To test and validate the design in an actual wind tunnel.
