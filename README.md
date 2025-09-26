# Vex IRB Adapter
This project is an adapter for the SJSU E10 Infrared Receiver Board to allow communication with Vex 5 brain through use of a RS485 tranceiver and an RJ11 port (commonly called a smart port on Vex side). There is also a QWIIC connector on board to allow use of a [HuskyLens AI camera](https://www.dfrobot.com/product-1922.html) communicating over I2C. 

The PCB design is open source and can be found on [OSHWLab](https://oshwlab.com/libhal/vex-adapter). The power regulator is outsourced from [Pololu](https://www.pololu.com/product/5592) because it is surprisingly cheaper and more reliable than what we can make. We have also designed a [3D printed enclosure on OnShape](https://cad.onshape.com/documents/ee69ea771b3426ac97776444/w/45ac0aa7c9bd20a1984d74b6/e/4c369e0476760c7468856e8b?renderMode=0&uiState=68d6b1c8a8c68f57f9c83bdb) with heated inserts to allow quick assembly and disassembly.

The code in this repo contains the firmware for the adapter, as well as examples for how to write code for the vex. The `/vex-code/` folder contains templates and examples on how to write code using VS Code with the [VEX extension](https://www.vexrobotics.com/vexcode/vscode-extension), as well as single file examples that are created and used with the [Vex Web Editor](https://codev5.vex.com/).

# Getting Started
Adapters must be flashed in order to work properly. We **highly recommend** using the Webflasher method as it is the easiest way.

The USB-C port needed for flashing is covered by the top case as to not encourage students to plug whatever they please into the port. Three screws on the bottom of the case can be unscrewed to release the top from the bottom.

TODO: pictures and explain USB is under top cover

## Flashing Adapter Via Webflasher (Recommended)
TODO: add link to webflasher when done

Connect adapter to computer via USB-C port and go to the [E10 adapter webflasher]().

Click the connect and flash button and select the adapter in the pop-up.

If you are unsure which device to choose, unplug and replug the adapter in while the pop-up is open.

Wait until device is fully flashed, then you are free to disconnect device and use.

## Building and Flashing Manually
Before getting started, if you haven't used libhal before, follow the
[Getting Started](https://libhal.github.io/latest/getting_started/) guide.

Clone or download the repo, then `cd` into the repo from a terminal.

### 📥 Installing Platform Profiles
```bash
conan config install -sf conan/profiles/v1 -tf profiles https://github.com/libhal/libhal-arm-mcu.git
```
The device profiles only has half of the information. The other half needed to build an application is the compiler profile. Compiler profiles are used to instruct the conan+cmake build system on the compiler to use for the build.

```bash
conan config install -sf conan/profiles/v1 -tf profiles https://github.com/libhal/arm-gnu-toolchain.git
```

### 🏗️ Building Application
```bash
conan build . -pr stm32f103c8 -pr arm-gcc-12.3
```

### ⚡Flashing device
```bash
stm32loader -e -w -v -B -p <device com> build/stm32f103c8/MinSizeRel/app.elf.bin
```

# Project Templates and Helper Class
Students can be given the templates as a starting point to have access to our helper class which provides functions to make accessing the data easier. 

The `single-file-code` template can be opened in the web editor by selecting "File" then "Open" and selecting the template.

To use the `vs-code-template`, open the Vex extension tab in VS Code, select "Import Project", and select the `vs-code-template` folder. 

## Requesting and Receiving Data With Helper Class
The templates provide a helper class and functions to assist in asking for data. They each return a struct containing the needed data and are explained below.

### Low Frequency IR Data
Function Call:
```c++
irb_adapter::adapter board(1);
auto data = board.get_low_freq_data();
```
Returns struct:
```c++
// defaults to 0 in case of bad read
struct low_freq_data {
  uint8_t low_freq_photo_diode = 0;
  uint8_t low_freq_value = 0;
};
```

### High Frequency IR Data
Function Call:
```c++
irb_adapter::adapter board(1);
auto data = board.get_hi_freq_data();
```
Returns struct:
```c++
// defaults to 0 in case of bad read
struct hi_freq_data {
  uint8_t hi_freq_photo_diode = 0;
  uint8_t hi_freq_value = 0;
};
```

### AI Camera Data
Function Call:
```c++
irb_adapter::adapter board(1);
auto data = board.get_cam_data();
```
Returns struct:
```c++
// defaults to 0 in case of bad read or no object detected
struct camera_data {
  uint16_t x_center = 0;
  uint16_t y_center = 0;
  uint16_t block_width = 0;
  uint16_t block_height = 0;
};
```

### Example Data Usage
```c++
irb_adapter::adapter board(1);
auto data = board.get_hi_freq_data();

uint8_t pd_number = data.hi_freq_photo_diode;
```

## Requesting Data Without Helper Class
The adapter can report three different types of data, low frequency IR reading, high frequency IR reading, and AI camera data. The Vex code must send a single byte requesting the data type to receive and will receive a set of bytes containing the data. The send and response bytes are explained in the table below. Checksums are calculated by adding all received bytes and taking the lower 8 bits of the result. If checksums do not match, data is either missing or corrupted.

| Send Byte      | Response Bytes     |
| ------------- | ------------- |
| `l` | [ PD Number ] , [ Intensity (0 - 127) ] , [ Checksum ] |
| `h` | [ PD Number ] , [ Intensity (0 - 127) ] , [ Checksum ] |
| `c` | [ X Center Upper Bits ] , [ X Center Lower Bits ] , [ Y Center Upper Bits ] , [ Y Center Lower Bits ] , [ Width Upper Bits ] , [ Width Lower Bits ] , [ Height Upper Bits ] , [ Height Lower Bits ] , [ Checksum ] |
