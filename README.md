
## BUILD
west build -b nrf5340dk_nrf5340_cpuapp

## FLASH
west flash --runner jlink

## PIN CONVERTION
  P1.1 = (0*32) + 1 = 33

### DEVICES
```sh
//GPS
&uart1 {
  compatible = "nordic,nrf-uarte";
  current-speed = <9600>;
  status = "okay";
  rts-pin = <11>;//SB28
  tx-pin = <33>;//P1.01 - SB30
  rx-pin = <32>;//P1.00 - SB29
};
//AUDIO FX
&uart2 {
  compatible = "nordic,nrf-uarte";
  current-speed = <9600>;
  status = "okay";
  //rts-pin = <19>;//P0.19 - SB51
  tx-pin = <40>;//P1.08 - SB53
  rx-pin = <41>;//P1.09 - SB53
};
//UV
&i2c3 {
  reg = <0xc000 0x1000>;
  clock-frequency = <100000>;
  interrupts = <12 1>;
  status = "okay";
  label = "I2C_3";
  compatible = "nordic,nrf-twim";
  sda-pin = < 34 >;//P1.02 - SDA
  scl-pin = < 35 >;//P1.03 - SCL
 };
```



## Judging criteria

Your project will be judged according to the rubric below.

### Project Documentation (30 points)
    Story/Instructions - Show how you created your project, including images, screenshots, and a video demonstration of your solution working as intended. Ask yourself: “If I were a beginner reading this project, would I understand how to recreate it?”
### Complete BOM (10 points)
    Detail the hardware, software and/or tools used.
### Schematics (10 points)
    Draw your circuit diagrams using software like Fritzing.org and/or take detailed photographs.
### Code & Contribution (20 points)
    Include working code with helpful comments.
### Creativity (30 points)
    Your idea doesn’t have to be original, but it should be creative (a fresh take on an old idea is often worth as much as a brand new idea)
### Demo video (BONUS - 15 points)
    Include a demo video showing your solution and get 15 bonus points! The length of the video can be between 5 seconds and 1 minute long. Nordic reserves the right to use the video to promote the hardware after the contest is over