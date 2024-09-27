### SPI read/write tool

Below are the pins configured as the SPI master and SPI slave:

| Line        | SPI master  | SPI slave   |
| ----------- | ----------- | ----------- |
| SCK         | P0.31       | P1.01       |
| MOSI        | P0.30       | P1.00       |
| MISO        | P0.29       | P1.02       |
| CS          | P0.28       | P1.03       |

Setup:  
1. Connect wires from SPI master pins to SPI slave pins on the nRF52840 DK board
2. Create a build configuration and add `nrf52840_nrf52840.overlay` file    
3. Build and flash the application into the Nordic board   
4. Open and listen to the `/dev/ttyACM0`(can be different depending on which serial port is assigned) serial port to access shell

Usage:
```
spi list
spi read
spi write <address> <data>
```
