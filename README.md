# Cacao Pulp Detection and Sorting Automation

Raspberry Pi–based conveyor sorter that uses a USB webcam and a YOLOv8n TFLite model to classify cacao pulp as **accept** (opaque milky white / light off-white) or **reject** (brown, grey, yellow-green, translucent, black-streaked, or moldy). Confirmed rejects are kicked off the belt by a servo arm. A 16×2 I2C LCD shows live status.

---

## Hardware

| Component | Details |
|-----------|---------|
| Raspberry Pi 4B (4 GB) | Main controller |
| USB Webcam (Sonix "Webcam: Webcam") | MJPG 640×480 at 30 FPS, blue USB 3.0 port |
| 16×2 I2C LCD (PCF8574 backpack) | Address 0x27 |
| SG90 servo | Reject kicker arm |
| Relay module (active LOW) | Conveyor motor (12 V DC) |
| Start button | Momentary push button |
| Safe Shutdown button | Momentary push button |

---

## Wiring (BCM numbering)

| GPIO | Pin | Function |
|------|-----|----------|
| 2 | 3 | I2C SDA → LCD |
| 3 | 5 | I2C SCL → LCD |
| 12 | 32 | Servo signal (PWM) |
| 17 | 11 | Safe Shutdown button → GND |
| 23 | 16 | Relay IN (conveyor, active LOW) |
| 27 | 13 | Start button → GND |

**Buttons:** one leg to the GPIO pin, one leg to GND. No resistor or VCC needed — internal pull-ups are used.

**Servo:** powered from a buck converter set to 5.0 V (not from the Pi's 5 V rail). Connect the buck converter GND to Pi GND.

**Relay:** logic side powered from the Pi's 5 V rail. Motor side powered from the 12 V supply.

---

## Raspberry Pi Setup

### 1. Enable interfaces

```bash
sudo raspi-config
```

Go to **Interface Options** and enable:
- **I2C** (for the LCD)

Reboot after enabling:

```bash
sudo reboot
```

Verify I2C and the LCD address:

```bash
i2cdetect -y 1
# Should show "27" in the grid
```

### 2. Install system packages

```bash
sudo apt update && sudo apt upgrade -y
sudo apt install -y i2c-tools v4l-utils
```

### 3. Create the Python virtual environment

```bash
cd ~
python3 -m venv cacao_pulp/venv313 --system-site-packages
source cacao_pulp/venv313/bin/activate
pip install --upgrade pip
pip install numpy opencv-python RPi.GPIO RPLCD ai-edge-litert
```

Verify:

```bash
python -c "from ai_edge_litert.interpreter import Interpreter; print('TFLite OK')"
python -c "import RPi.GPIO; print('GPIO OK')"
python -c "import RPLCD; print('RPLCD OK')"
python -c "import cv2; print('OpenCV OK')"
```

### 4. Enable the Safe Shutdown button (kernel overlay)

Add this line to `/boot/firmware/config.txt` under the `[all]` section:

```bash
sudo nano /boot/firmware/config.txt
```

```
[all]
dtoverlay=gpio-shutdown,gpio_pin=17
```

Also uncomment or add this line to enable I2C:

```
dtparam=i2c_arm=on
```

Reboot:

```bash
sudo reboot
```

Confirm the overlay loaded:

```bash
dmesg | grep shutdown_button
# Should show: input: soc:shutdown_button@11 as /devices/...
```

A single press of the Safe Shutdown button (GPIO17) will now trigger a clean shutdown at the kernel level.

### 5. Clone the repository

```bash
cd ~
git clone https://github.com/gabgonzales-ph/Cacao-Pulp-Detection-and-Sorting-Automation.git cacao_pulp
cd cacao_pulp
```

### 6. Add the model

Copy the trained TFLite model to:

```
~/cacao_pulp/models/cacao_pulp_model_v1_float32.tflite
```

The script runs without detection if the model file is missing — the conveyor, buttons and LCD still work.

### 7. Install and enable the systemd service

```bash
sudo tee /etc/systemd/system/cacao-sorter.service << 'EOF'
[Unit]
Description=Cacao Pulp Sorting System
After=multi-user.target

[Service]
Type=simple
User=cacaoadmin
WorkingDirectory=/home/cacaoadmin/cacao_pulp
ExecStartPre=/bin/sleep 8
ExecStart=/home/cacaoadmin/cacao_pulp/venv313/bin/python /home/cacaoadmin/cacao_pulp/main.py
ExecStop=/bin/sh -c 'pinctrl set 23 op dh'
Restart=on-failure
RestartSec=5
KillSignal=SIGTERM
TimeoutStopSec=15

[Install]
WantedBy=multi-user.target
EOF

sudo systemctl daemon-reload
sudo systemctl enable cacao-sorter.service
sudo systemctl start cacao-sorter.service
```

Check status:

```bash
sudo systemctl status cacao-sorter.service --no-pager
```

---

## Running Manually

Stop the service first (camera and GPIO cannot be shared):

```bash
sudo systemctl stop cacao-sorter.service
cd ~/cacao_pulp
source venv313/bin/activate
python main.py
```

To show the live camera preview (requires a display or VNC):

```bash
SHOW_PREVIEW=1 python main.py
```

Press **Q** in the preview window or **Ctrl+C** in the terminal to stop.

---

## Pin State Check

```bash
pinctrl get 2
pinctrl get 3
pinctrl get 12
pinctrl get 17
pinctrl get 23
pinctrl get 27
```

---

## Troubleshooting

| Symptom | Fix |
|---------|-----|
| LCD shows "Init Failed" | Check `~/cacao_pulp/error.log`. I2C may not be enabled or LCD is not wired. |
| LCD blank, no address at 0x27 | Run `i2cdetect -y 1`. Check SDA/SCL wiring and VCC/GND on the LCD. |
| "Could not open webcam" | Stop the service first: `sudo systemctl stop cacao-sorter.service`. Then check `sudo lsof /dev/video0`. |
| Conveyor does not turn on | Check relay wiring (use NO terminal, not NC). Verify `ACTIVE_LOW = True` in `components/motors.py`. |
| Shutdown button does nothing | Check `dmesg | grep shutdown_button`. If missing, confirm `dtoverlay=gpio-shutdown,gpio_pin=17` is under `[all]` in `config.txt` and reboot. |
| Script exits immediately | Check `tail -20 ~/cacao_pulp/error.log` for the cause. |
| Camera works manually but not at boot | Increase `ExecStartPre` sleep from 8 to 15 seconds in the service file, then `sudo systemctl daemon-reload`. |
