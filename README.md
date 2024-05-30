# Esp-Latency-Test

Code for performing end-to-end latency test for inputs. Uses gpio output to pull
a button low, then uses a photo-transistor to measure when the screen changes
and computes the time it took.

This repository also contains an [`analysis.py`](./analysis.py) script, which can be used to plot
a histogram of latency values that are measured from the system.

## Hardware Needed

1. ESP32 dev board (code defaults to esp32s3), such as QtPy ESP32S3.
2. Dupont wires to connect to button on controller (patch into button and gnd
   signal).
3. Photo-diode for measuring the brightness / light of the screen. I used
   [Amazon 3mm flat head PhotoDiode](https://www.amazon.com/dp/B07VNSX74J).
4. Resistor (1k-10k) from photodiode output to ground.

## Use

It's recommended to use the
[uart_serial_plotter](https://github.com/esp-cpp/uart-serial-plotter) after
flashing to monitor and plot the latency values in real time. If you do this,
you can then also save the resultant output to a text file.

This text file can be loaded and parsed by the [`analysis.py`](./analysis.py)
script.

### Real-time Plotting

You can use the
[uart_serial_plotter](https://github.com/esp-cpp/uart-serial-plotter) to plot
the latency values in real time.

``` sh
# follow setup / use instructions in esp-cpp/uart_serial_plotter repo
➜  uart_serial_plotter git:(master) $ source env/bin/activate
(env) ➜  uart_serial_plotter git:(master) $ python src/main.py
```

It will automatically find and open the serial port with the esp32 attached. If
there are multiple, you can use the `Serial` menu. to select another port.

If you want to save the recorded data to a file, you can use `File > Export UART
Data` command to save the data to a `txt` file.

### Analysis

#### Setup

These setup steps only need to be run the first time you set up the python
environment.

``` sh
# create the environment
➜  esp-latency-test git:(main) $ python3 -m venv env

# activate the environment
➜  esp-latency-test git:(main) $ source env/bin/activate

# install the dependencies (matplotlib, numpy)
(env) ➜  esp-latency-test git:(main) $ pip install -r requirements.txt
```

#### Running

Any time you have a text file of csv data (such as what comes from the esp32
code), you can run the python script on it to generate a histogram.

``` sh
# This will run an interactive plot with matplotlib
(env) ➜  esp-latency-test git:(main) $ python ./analysis.py tests/2024-05-30.txt

# This will simply save the output to the provided png file (destination folder must exist if provided)
(env) ➜  esp-latency-test git:(main) $ python ./analysis.py tests/2024-05-30.txt --output output/2024-05-30.png

# you can also specify your own title
(env) ➜  esp-latency-test git:(main) $ python ./analysis.py tests/2024-05-30-15ms-wake.txt --output output/2024-05-30-15ms-wake.png --title "Latency Histogram"
```

## Cloning

Since this repo contains a submodule, you need to make sure you clone it
recursively, e.g. with:

``` sh
git clone --recurse-submodules git@github.com:finger563/esp-latency-test
```

Alternatively, you can always ensure the submodules are up to date after cloning
(or if you forgot to clone recursively) by running:

``` sh
git submodule update --init --recursive
```

## Configuring

You can configure a few parts of the project, such as the GPIO for the button,
the ADC for the sensor, and the thresholds to be used.

To configure the project, run

``` sh
idf.py menuconfig
```

## Build and Flash

Build the project and flash it to the board, then run monitor tool to view serial output:

```
idf.py -p PORT flash monitor
```

(Replace PORT with the name of the serial port to use.)

(To exit the serial monitor, type ``Ctrl-]``.)

See the Getting Started Guide for full steps to configure and use ESP-IDF to build projects.

## Output

Example screenshot of the console output from this app:

![CleanShot 2024-05-30 at 16 16 43](https://github.com/finger563/esp-latency-test/assets/213467/e69c99bf-af8e-42cb-8903-8c4b001d8759)

Example histogram generated from the analysis tool:

![image](https://github.com/finger563/esp-latency-test/assets/213467/0835a8ce-d152-40a8-823a-664772681a63)

