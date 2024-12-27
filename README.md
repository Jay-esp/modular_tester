# modular_tester

Modular Test Equipment Rack

![IMG_4679](https://github.com/user-attachments/assets/2f432137-6b09-4a2c-b6a2-88ec036765db)

A little intro….

In the late 1970’s I built some test equipment from Elektor magazine, Radio Bulletin and my own designs because I could not afford to buy the real stuff. Today I’m retired and own a lot of 70’s/80’s/90’s professional equipment but I like to design and build my own.

Building equipment the classic way (put parts on a PCB, built it in a box, add a power supply, write code,…) is a bit restrictive, you can’t simply add things. Hence I was looking for a way to build equipment that is expandable and has shared power supplies. I found inspiration from Eurorack modules used in the electronic music world and a 70’s Philips modular mixing panel that you could build to your own taste from a range of kits.

I mill the front panels and mounting hardware in aluminum on my own design CNC router.

These days professional PCB’s are very affordable if you pick the right supplier and shipping option, processor modules such as the ESP32 devkit are very powerful and cheap, together with the Arduino environment a great enabler.

My first rack is already full, i have a second one ready and a third for development purposes.

The modules that are completed:

Signal Tracer

CB microphone tester

ESR meter

Capacitor outer foil checker

VI (Huntron) tracker

Short seeker

RF probe

DMX tester

Clock/timer

TC1 component tester

Ring (flyback/lopt) tester

Battery tester

LED tester

Frequency counter

Crystal tester

In progress:

FM transmitter
White/pink/brown noise generator
...

The concept…

A rack in MDF with shared power supplies, the rack and module construction is documented in modular_tester_rack_construction.pdf.

Module with a front and supporting panel, brackets and a PCB, the documentation consists of the circuit diagram, PCB view, gerber and drill files, ESP32 software, designs for front and back panels, artwork for front panel and user guide/specific instructions.

Instructions on how to setup the Arduino software development environment and to program the ESP32 are in modular_tester_programming_instructions_ESP32.pdf.

I will publish videos on YouTube, the introduction video: https://youtu.be/arFC4ruuN68

This repository will be a running project, I will add modules and YouTube videos as I go along.

Disclaimer: although all designs have been built and tested, provided as-is, no warranty or liability, use at your own risk. I am not responsible for what you build or how you use it, a decent level of knowledge of electronics and soldering skills are implied. I do not sell kits.

You can use the designs as-is or as an inspiration for your own designs. If you build something, let me know…

Suggestions for future modules welcome. If some wizard wants to collaborate on a few FPGA based designs, I could use a little help (Lattice MachxO2).

I don’t drink coffee but you can always buy me a beer :)





