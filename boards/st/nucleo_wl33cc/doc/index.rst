.. zephyr:board:: nucleo_wl33cc

Overview
********

The STM32WL33CC1 Nucleo-64 boards based on the MB1801 mezzanine board and MB2029 MCU RF board
(NUCLEOWL33CC1 and NUCLEO-WL33CC2 order codes) embed the STM32WL33CCV6 sub-GHz application processor.
This high‑performance and low‑power application processor can operate in 433, 868, and 915 MHz bands.
The ARDUINO® Uno V3 connectivity support and the ST morpho headers provide an easy means of expanding the functionality
of the STM32 Nucleo open development platform with a wide choice of specialized shields.
The STM32WL33CC1 Nucleo-64 boards are supplied with a dedicated software package, HAL library, and various packaged
software examples available with the STM32CubeWL3 MCU Package.
The boards are declined in two product variants with dedicated front ends tuned for specific frequency bands.

Hardware
********

Ultra-low-power wireless STM32WL33CCV6 microcontroller based on the Arm® Cortex®‑M0+ core, with
256 Kbytes of flash memory and 32 Kbytes of SRAM in a VFQFPN48 package featuring:
   – Ultra-low-power MCU
   – Sub-GHz transceiver with IPD front end optimized for 413‑479 MHz or 826‑958 MHz frequency
bands, supporting OOK, ASK, 2(G)FSK, 4(G)FSK, D‑BPSK, and DSSS modulations
   – Compatible with proprietary and standardized wireless protocols such as WM-Bus, Sigfox™, mioty,
KNX-RF, and IEEE 802.15.4g
   – Low-power autonomous wake-up receiver
• Delivered with SMA antenna
• Three user LEDs
• Three user and one reset push-buttons
• Board connectors:
   – USB Type-C®
   – ARDUINO® Uno V3 expansion connector
   – ST morpho extension pin headers for full access to all MCU I/Os
• Flexible power-supply options: ST-LINK USB VBUS or external sources
• On-board STLINK-V3EC debugger/programmer with USB re-enumeration capability: mass storage, Virtual
COM port, and debug port
• Comprehensive free software libraries and examples available with the STM32CubeWL3 MCU Package
• Dedicated software tool to control and test radio transceiver
• Support of a wide choice of Integrated Development Environments (IDEs) including IAR Embedded
Workbench®, MDK-ARM, and STM32CubeIDE

More information about STM32WL33CCV6 can be found here:

- `STM32WL33CCV6 on www.st.com`_
- `STM32WL33CC1 reference manual`_


Supported Features
==================

.. zephyr:board-supported-hw::

Radio features supported only by using HAL drivers directly.

Connections and IOs
===================

Default Zephyr Peripheral Mapping:
----------------------------------

- USART1 TX/RX       : PA9/PA8 (ST-Link Virtual COM Port)
- BUTTON (B1)        : PA0
- BUTTON (B2)        : PA11
- BUTTON (B3)        : PB15
- LED (LD1/BLUE)     : PA14
- LED (LD2/GREEN)    : PB4
- LED (LD3/RED)      : PB5

For more details, please refer to the `STM32WL33CC1 Nucleo-64 board User Manual`_.

Programming and Debugging
*************************

.. zephyr:board-supported-runners::

Nucleo STM32WL33CC1 board includes an ST-LINK-V3EC embedded debug tool interface.

Applications for the ``nucleo_wl33cc`` board target can be built and flashed
in the usual way (see :ref:`build_an_application` and :ref:`application_run`
for more details).

Flashing
========

The board is configured to be flashed using the west `STM32CubeProgrammer`_ runner,
so :ref:`it must be installed <stm32cubeprog-flash-host-tools>` beforehand.

Alternatively, OpenOCD can also be used to flash the board using the
``--runner`` (or ``-r``) option:

.. code-block:: console

   $ west flash --runner openocd

Flashing an application to Nucleo STM32WL33CC1
----------------------------------------

Connect the Nucleo STM32WL33CC1 to your host computer using the USB port,
then run a serial host program to connect with your Nucleo board:

.. code-block:: console

   $ minicom -D /dev/ttyACM0

Now build and flash an application. Here is an example for
:zephyr:code-sample:`hello_world`.

.. zephyr-app-commands::
   :zephyr-app: samples/hello_world
   :board: nucleo_wl33cc
   :goals: build flash

You should see the following message on the console:

.. code-block:: console

   Hello World! nucleo_wl33cc/stm32wl33

Usage of the pyOCD runner requires installation of an additional target pack.
This can be done using the following commands:

.. code-block:: console

   $ pyocd pack update
   $ pyocd pack install stm32wl3

Debugging
=========

You can debug an application in the usual way.  Here is an example for the
:zephyr:code-sample:`hello_world` application.

.. zephyr-app-commands::
   :zephyr-app: samples/hello_world
   :board: nucleo_wl33cc
   :maybe-skip-config:
   :goals: debug

.. _`Nucleo STM32WL33CC1 webpage`:
   https://www.st.com/en/evaluation-tools/nucleo-wl33cc1.html

.. _`WL33CC on www.st.com`:
   https://www.st.com/en/microcontrollers-microprocessors/stm32wl33cc.html

.. _`STM32WL33 reference manual`:
   https://www.st.com/resource/en/reference_manual/rm0511-stm32wl30xx31xx33xx-armbased-wireless-mcus-with-subghz-radio-solution-stmicroelectronics.pdf

.. _`Nucleo WB05KZ board User Manual`:
   https://www.st.com/resource/en/user_manual/um3418-stm32wl33-nucleo64-boards-mb1801-and-mb2029-stmicroelectronics.pdf

.. _STM32CubeProgrammer:
   https://www.st.com/en/development-tools/stm32cubeprog.html
