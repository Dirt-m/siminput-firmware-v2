import usb_hid
import supervisor
import json
import usb_cdc
import storage

# Read device identity from config.
# IMPORTANT — pid must differ from the default CircuitPython HID PID (0x80F4).
# Windows caches the joystick name per VID+PID in the registry the first time it
# sees a device; changing only the product string has no effect on that cached
# entry. Using a unique PID guarantees Windows creates a fresh registry entry
# that picks up our product name correctly.
_name = "SimInput Button Box"
_vid  = 0x239A   # Adafruit VID — fine for personal/custom projects
_pid  = 0xF000   # Default SIMINPUT PID, distinct from any CircuitPython default

try:
    with open("config.json") as _f:
        _dev  = json.load(_f).get("device", {})
        _name = _dev.get("name", _name)
        _pid  = _dev.get("pid",  _pid)
except Exception:
    pass

# Fixed 128-button + 8-axis descriptor (Option A: max capacity, always the same).
# Report layout (32 bytes total):
#   Bytes  0-15 : 128 buttons, 1 bit each (B1 = bit 0 of byte 0)
#   Bytes 16-31 : 8 axes, 16-bit unsigned each (0-65535), little-endian
#                 Axis slots 1-8 map to X, Y, Z, Rx, Ry, Rz, Slider, Dial
GAMEPAD_DESCRIPTOR = bytes((
    0x05, 0x01,              # Usage Page (Generic Desktop)
    0x09, 0x04,              # Usage (Joystick)
    0xA1, 0x01,              # Collection (Application)

    # 128 buttons
    0x05, 0x09,              # Usage Page (Button)
    0x19, 0x01,              # Usage Minimum (1)
    0x29, 0x80,              # Usage Maximum (128)
    0x15, 0x00,              # Logical Minimum (0)
    0x25, 0x01,              # Logical Maximum (1)
    0x75, 0x01,              # Report Size (1 bit)
    0x95, 0x80,              # Report Count (128)
    0x81, 0x02,              # Input (Data, Var, Abs)

    # 8 axes, 16-bit unsigned (0-65535)
    0x05, 0x01,              # Usage Page (Generic Desktop)
    0x09, 0x30,              # Usage (X)
    0x09, 0x31,              # Usage (Y)
    0x09, 0x32,              # Usage (Z)
    0x09, 0x33,              # Usage (Rx)
    0x09, 0x34,              # Usage (Ry)
    0x09, 0x35,              # Usage (Rz)
    0x09, 0x36,              # Usage (Slider)
    0x09, 0x37,              # Usage (Dial)
    0x15, 0x00,              # Logical Minimum (0)
    0x27, 0xFF, 0xFF, 0x00, 0x00,  # Logical Maximum (65535)
    0x75, 0x10,              # Report Size (16 bits)
    0x95, 0x08,              # Report Count (8)
    0x81, 0x02,              # Input (Data, Var, Abs)

    0xC0,                    # End Collection
))

_gamepad = usb_hid.Device(
    report_descriptor=GAMEPAD_DESCRIPTOR,
    usage_page=0x01,
    usage=0x04,
    report_ids=(0,),
    in_report_lengths=(32,),
    out_report_lengths=(0,),
)

# Both calls must come BEFORE usb_hid.enable().
#
# supervisor.set_usb_identification sets the USB device-level product string.
# usb_hid.set_interface_name sets the HID interface name — this is the string
# that Windows game controllers panel and WebHID actually display. It was added
# in CircuitPython 9.0.2; update CP if the name still shows as CircuitPython HID.
try:
    supervisor.set_usb_identification(
        manufacturer="SIMINPUT",
        product=_name,
        vid=_vid,
        pid=_pid,
    )
    print("USB device ID:", _name, "VID", hex(_vid), "PID", hex(_pid))
except Exception as e:
    print("set_usb_identification failed:", e)

try:
    usb_hid.set_interface_name(_name)
    print("USB HID interface name:", _name)
except Exception as e:
    print("set_interface_name failed (requires CircuitPython 9.0.2+):", e)

usb_cdc.enable(console=True, data=True)
storage.disable_usb_drive()
storage.remount("/", readonly=False)

usb_hid.enable((_gamepad,))
