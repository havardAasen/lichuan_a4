# Lichuan A4 servo driver

An userspace program that interfaces the Lichuan A4 servo drive to LinuxCNC
HAL, using RS485 Modbus RTU.

## Build and install

I'm assuming you already have set up LinuxCNC, if not, go to their website
<https://linuxcnc.org> to get started.

To download and build the driver, we need some packages:

- `build-essential`, C++ compiler and *make*
- `cmake`, configure the project
- `git`, download this Git repository
- `libmodbus-dev`, development files for Modbus
- `linuxcnc-uspace-dev`, development files for LinuxCNC

``` shell
sudo apt-get install build-essential cmake git libmodbus-dev linuxcnc-uspace-dev
git clone https://github.com/havardAasen/lichuan_a4.git
cd lichuan_a4
mkdir build
cmake -S . -B build/
cmake --build build/
sudo cmake --install build/
```

## Documentation and usage

The man page `lichuan_a4.1` describes the parameter to adjust on the
Lichuan A4 servo drive. It also lists the different command-line options if you
need to customize anything regarding Modbus The man-page also lists the
pins and signals which is used with LinuxCNC.

The provided file `custom.hal` is an example on how to create the signals
and connect the pins to LinuxCNC.

## Testing

If you want to test the servo drive, you can use one of the sample configurations
that comes shipped with LinuxCNC.

- Execute the steps in **Build and install**, ending with installing
  the binaries.
- Open LinuxCNC and choose one of the sample configurations.
  `Sample Configurations -> sim -> axis`
  choose one of `axis`, `axis_9axis` or `axis_mm`.
- Say yes to copy the files to your home folder.
- Exit LinuxCNC
- Copy the `custom.hal` file, from the repository into the newly created
  configuration folder
- In the configuration folder, edit the axis*.ini file you wish to use.
  It's three of these, but you only need to use one of them.
- Go to `HAL` section and comment out `HALFILE = sim_spindle_encoder.hal`.
- Continuing in the `HAL` section, add `HALFILE = custom.hal` as the last entry.

## License

This software is released under the **GPLv2** license. See the file `COPYING`
for more information.
