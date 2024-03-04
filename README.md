# I2C OLED with ESP-IDF and Wokwi Simulation

[esp_lcd](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/lcd.html) supports I2C interfaced OLED LCD, whose color depth is usually 1bpp.

This example shows how to make use of the SSD1306 panel driver from `esp_lcd` component to facilitate the porting of LVGL library. In the end, example will display a scrolling text on the OLED screen.

## Building

We recommend using the [ESP-IDF VSCode extension](https://marketplace.visualstudio.com/items?itemName=espressif.esp-idf-extension) to build the project. Alternatively, you can use the command line: `idf.py build`.

## Simulation

To simulate this project, install [Wokwi for VS Code](https://marketplace.visualstudio.com/items?itemName=wokwi.wokwi-vscode). Open the project directory in Visual Studio Code, press **F1** and select "Wokwi: Start Simulator".

## Debugging

The project is already set up for debugging. To debug the project, first start Wokwi in debug mode (press **F1** and select "Wokwi: Start Simulator and Wait for Debugger"). Then press **F5** to start the debugger. You can set breakpoints, step through the code, and inspect variables.

The debugger is configured to use TCP port 3333. If this port is already in use on your computer, you can change it in the `.vscode/launch.json` file (`miDebuggerServerAddress`) and in `wokwi.toml` (`gdbServerPort`).

Note that the debugger setup requires the ESP-IDF extension to be installed in VS Code. If you don't have the ESP-IDF extension, you can manually set `miDebuggerServerAddress` in `.vscode/launch.json` to point to your local installation of the `xtensa-esp32-elf-gdb` debugger (it's usually installed in the esp tools directory, under `tools/xtensa-esp-elf-gdb/<version>/xtensa-esp-elf-gdb/bin`).
