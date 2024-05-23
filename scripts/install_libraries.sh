echo "Updating apt-get"
sudo apt update -y
sudo apt-get update -y

echo "Installing the i2c library"
sudo apt install -y libi2c-dev i2c-tools

echo "Installing pip3"
sudo apt install -y python3-pip

echo "Installing the smbus library"
sudo apt install -y pip3 install smbus
