
# Install libraries

You can clone the repository by copying and pasting this command:

```bash
git clone --recursive https://github.com/vertueux/smov
```

You currently need to install these libraries to ensure that SMOV can function:

* `libi2c-dev` & `i2c-tools` (for controlling the servos)
* `smbus`  (for controlling the LCD panel)

To install them, simply copy and paste the following commands:

```bash
cd ~/smov/scripts
chmod +x install_libraries.sh
./install_libraries.sh
```

**Next step**: [Configure ports](configure_ports.md)
