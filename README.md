# SpacecraftSimulator

To use the simulator you must open and execute the file "SatSimulator.py". This file initializes the variables read from the Data folder and the .ini files.

The .ini files can be modified to generate different scenarios, dynamics and change the properties of the components.


## Requirements

### Required Libraries
 - Numpy
 - jpl-ephem

### Required files
 - The file de430.bsp must be downloaded and added to the directory:
"\Dynamics\CelestialBody\cspice\generic_kernels\spk\planets\"

 - Download:
 https://naif.jpl.nasa.gov/pub/naif/generic_kernels/spk/planets/


## Installation
clone the repository in a local folder

```bash
git clone https://github.com/spel-uchile/SpacecraftSimulator
```
install python requirements.

```bash
pip3 install -r requirements.txt
```
install other dependencies
```bash
python3 install.py
```
run simulation with file SatSimulator.py
```bash
python3 SatSimulator.py
```

***Contact***

- els.obrq@gmail.com
