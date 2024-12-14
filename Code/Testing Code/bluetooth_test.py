from pyb import UART, Pin


def test_funct():
    print("test printout")

if __name__ == '__main__':
    # Make a serial port object from the UART class
    BT_ser = UART(3, 9600)

    # Deconfigure default pins
    Pin(Pin.cpu.C4,  mode=Pin.ANALOG)     # Set pin modes back to default
    Pin(Pin.cpu.C5,  mode=Pin.ANALOG)

    # Configure the selected pins in coordination with the alternate function table
    Pin(Pin.cpu.C4,  mode=Pin.ALT, alt=7) # Set pin modes to UART matching column 7 in alt. fcn. table
    Pin(Pin.cpu.C5, mode=Pin.ALT, alt=7)

    pyb.repl_uart(BT_ser)

