# Embedded-State-Machine

This repository contains the embedded C code that initializes a state machine in an MSP432401R microprocessor. These states can be toggled on/off using buttons P1.1 and P1.4, and are reflected to the user using the LEDs.

The permitted states are as follows:<br>
State 1: P1.0 and P2.0 are both off<br>
State 2: P1.0 on, P2.0 off<br>
State 3: P1.0 off, P2.0 on<br>
State 4: P1.0 and P2.0 are both on<br>

When states are changed, the board attempts to transmit the new state over a UART connection, where it can be received through a port by any software listening. In addition to transmitting information, if another device sends information over the UART, this board can receive and process that command before relaying the new state back once again.

The code used to interact with the board can be found in the following repository: https://github.com/jonahgaudet/UART-Connection
