To build PhobGCC for the RP2040:

1. Install the compiler toolchain as per the [Pico documentation](https://datasheets.raspberrypi.com/pico/getting-started-with-pico.pdf).
2. Git clone the `pico-sdk` repository from https://github.com/raspberrypi/pico-sdk
3. Check out the master branch.
4. Run `git submodule update --init` (omit `--init` if you're not doing it for the first time)
5. Export the `pico-sdk` directory as an environment variable `export PICO_SDK_PATH=[directory containing]/pico-sdk`
6. Clone the PhobGCC-SW repository.
7. Uncomment the appropriate `#include` line for your board in `../common/phobGCC.h`.
8. `cd` into the `PhobGCC/rp2040` subdirectory inside the `PhobGCC-SW` folder.
9. `mkdir build && cd build`
10. `cmake ..`
11. `make`?
