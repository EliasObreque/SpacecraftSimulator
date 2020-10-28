# Install error state kalman filter
cd Components/Logic/Estimation
pip install -r requirements.txt
git clone -b feat-satsimulator-integration https://github.com/gdiazh/error_state_kalman_filter
cd ../../../

# Install generic kernels
if [ ! -f Dynamics/CelestialBody/cspice/generic_kernels/spk/planets/de430.bsp ]
then
    wget https://naif.jpl.nasa.gov/pub/naif/generic_kernels/spk/planets/de430.bsp
    mv de430.bsp Dynamics/CelestialBody/cspice/generic_kernels/spk/planets/
fi