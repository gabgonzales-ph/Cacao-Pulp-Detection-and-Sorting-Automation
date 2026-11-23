# Cacao Pulp Detection and Sorting Automation

A Raspberry Pi–based cacao pulp detection and sorting system using a USB webcam and a YOLOv8n TensorFlow Lite model. The system classifies cacao pulp as **good (accept)** or **reject**, controls a conveyor and feeding mechanism through relays, and uses an SG90 servo arm to remove confirmed rejects from the belt.

A 16×2 I2C LCD displays the system status and detection counts.

## Features

- Real-time cacao pulp detection using a YOLOv8n TFLite model.
- Two-class classification:
  - **Good:** Opaque milky white or light off-white pulp.
  - **Reject:** Brown, grey, yellow-green, translucent, black-streaked, or moldy pulp.
- Automatic reject removal using an SG90 servo.
- Relay-controlled conveyor, cracker, and dispenser.
- 16×2 I2C LCD status display.
- Start button and Safe Shutdown button.
- Emergency-stop input.
- Camera recovery handling.
- Automatic return to IDLE after 20 seconds without newly confirmed pulp during the belt stage.

---

## Hardware

| Component | Details |
|---|---|
| Raspberry Pi 4B (4 GB) | Main controller |
| USB Webcam | Sonix Webcam, MJPG 640×480 at 30 FPS |
| 16×2 I2C LCD | PCF8574 backpack, address `0x27` |
| SG90 servo | Reject kicker arm |
| Relay module | Active-LOW relay inputs |
| Conveyor motor | 12 V DC |
| Cracker motor | Relay-controlled |
| Dispenser motor | Relay-controlled |
| Start button | Momentary push button |
| Safe Shutdown button | Momentary push button |
| Emergency-stop button | Normally-open push button |

## GPIO Wiring

All GPIO numbers use **BCM numbering**.

| GPIO | Physical Pin | Function |
|---:|---:|---|
| GPIO 2 | 3 | I2C SDA → LCD |
| GPIO 3 | 5 | I2C SCL → LCD |
| GPIO 5 | 29 | Emergency stop → GND |
| GPIO 12 | 32 | Servo signal (PWM) |
| GPIO 17 | 11 | Safe Shutdown button → GND |
| GPIO 23 | 16 | Conveyor relay IN (active LOW) |
| GPIO 24 | 18 | Cracker relay IN (active LOW) |
| GPIO 25 | 22 | Dispenser relay IN (active LOW) |
| GPIO 27 | 13 | Start button → GND |

### Button wiring

Connect one leg of each button to its assigned GPIO pin and the other leg to GND.

The application uses internal pull-up resistors. No external resistor or VCC connection is required for these button inputs.

The emergency-stop input is active LOW and uses a normally-open button.

### Servo power

- Power the SG90 from a separate buck converter adjusted to 5.0 V.
- Do not power the servo directly from the Raspberry Pi's 5 V rail.
- Connect the buck converter GND to Raspberry Pi GND.
- Check that the servo arm can move freely without touching the conveyor or frame.

### Relay wiring

- Relay inputs are active LOW.
- The relay logic side is powered from the Raspberry Pi 5 V rail.
- Motor power comes from the appropriate external motor supply.
- Verify relay contact wiring and motor supply ratings before operation.

**Safety note:** The emergency-stop function currently depends on software. It is not a certified safety circuit. The normally-open GPIO input cannot detect a broken wire. For a proper emergency stop, install a suitably rated hardware circuit that physically interrupts motor and actuator power independently of the Raspberry Pi.

---

## Software Environment

The current Raspberry Pi environment uses:

- Raspberry Pi OS 64-bit (Debian Trixie)
- Python 3.13
- OpenCV
- NumPy
- RPi.GPIO
- RPLCD
- ai-edge-litert
- v4l-utils

The Python virtual environment is located at:

```text
~/cacao_pulp/venv313
```

## Raspberry Pi Setup

### 1. Enable I2C

Run:

```bash
sudo raspi-config
```

Go to:

**Interface Options → I2C → Enable**

Reboot:

```bash
sudo reboot
```

Verify the LCD address:

```bash
i2cdetect -y 1
```

The LCD should appear at address `0x27`.

### 2. Install system packages

```bash
sudo apt update
sudo apt upgrade -y
sudo apt install -y i2c-tools v4l-utils
```

### 3. Clone the repository

```bash
cd ~
git clone https://github.com/gabgonzales-ph/Cacao-Pulp-Detection-and-Sorting-Automation.git cacao_pulp
cd ~/cacao_pulp
```

### 4. Create the Python virtual environment

```bash
python3 -m venv ~/cacao_pulp/venv313 --system-site-packages
source ~/cacao_pulp/venv313/bin/activate
pip install --upgrade pip
pip install numpy opencv-python RPi.GPIO RPLCD ai-edge-litert
```

Verify the installations:

```bash
python -c "from ai_edge_litert.interpreter import Interpreter; print('TFLite OK')"
python -c "import RPi.GPIO; print('GPIO OK')"
python -c "import RPLCD; print('RPLCD OK')"
python -c "import cv2; print('OpenCV OK')"
```

### 5. Configure the Raspberry Pi boot settings

Enable I2C in:

```bash
sudo nano /boot/firmware/config.txt
```

Make sure this setting is present:

```ini
dtparam=i2c_arm=on
```

**Important: Do not enable this overlay:**

```ini
dtoverlay=gpio-shutdown,gpio_pin=17
```

GPIO17 is managed by the Python application for the Safe Shutdown button. The application handles the required 2-second button hold before initiating shutdown.

If the old overlay was previously added, remove it from `/boot/firmware/config.txt` and reboot.

### 6. Add the trained TFLite model

Place the current V3 TFLite model in the project's model directory.

The current model is the **V3 model (`v3.tflite`)**. Check the `MODEL_PATH` setting in `main.py` and place the model at the path configured there.

The V3 model was trained using 416×416 input images.

If the model file is missing or unavailable, detection will not operate correctly. Do not run automatic sorting without confirming that the intended model has loaded successfully.

### 7. Configure Safe Shutdown permissions

The system runs under the `cacaoadmin` service account. The application needs permission to execute:

```bash
sudo -n shutdown -h now
```

Configure a narrowly scoped sudoers rule for the service account using `visudo`. Do not grant unrestricted sudo access merely to enable shutdown.

### 8. Install the systemd service

Create the service file:

```bash
sudo nano /etc/systemd/system/cacao-sorter.service
```

Example configuration:

```ini
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
```

Make sure the repository, virtual environment, model path, and permissions match the actual installation.

Reload systemd and enable the service:

```bash
sudo systemctl daemon-reload
sudo systemctl enable cacao-sorter.service
sudo systemctl start cacao-sorter.service
```

Check its status:

```bash
sudo systemctl status cacao-sorter.service --no-pager
```

View service logs:

```bash
sudo journalctl -u cacao-sorter.service -f
```

---

## Operating Logic

The main operating sequence is:

```text
IDLE
  |
  | Press Start
  v
FEEDING
  |
  | Dispenser completes its cycle
  v
SORTING
  |
  | Confirmed pulp detected
  | Good pulp continues along the belt
  | Reject pulp triggers the servo kicker
  |
  | No newly confirmed pulp for 20 seconds
  v
IDLE
```

### Feeding stage

- The dispenser runs for the configured dispensing period.
- The current setting is `DISPENSER_SEC = 160.0`.
- When the dispenser turns off, the belt-stage empty-window timer begins.

### Belt and sorting stage

- The conveyor transports pulp through the camera detection area.
- Confirmed detections update the good and reject counts.
- Confirmed rejects activate the servo kicker.
- Pending or unconfirmed detections do not reset the empty-window timer.

### 20-second empty-window behavior

The current belt-stage logic uses:

```python
EMPTY_WINDOW_SEC = 20.0
```

- The timer starts when the dispenser turns off.
- Each newly confirmed good or reject count resets the timer.
- Yellow or pending detections do not reset the timer.
- If no new confirmed pulp is detected for 20 seconds, the system turns off the outputs and returns to IDLE.
- If the camera drops out during the belt stage, the empty window restarts at a full 20 seconds after camera recovery.
- `BELT_SEC` is retained as a fallback timeout when the camera or model is unavailable. It is no longer the normal belt-stage operating limit.

### Camera recovery

The application includes camera recovery handling. The intended behavior includes:

- Turning off relay outputs when no camera frame is received for the configured dropout period.
- Attempting to reopen the camera.
- Resuming operation after recovery.
- Aborting the batch if recovery exceeds the configured timeout.

Verify the actual deployed settings in `main.py` before relying on these timings.

---

## Servo Rejector

The SG90 servo is controlled through GPIO12.

Current intended servo settings:

| Setting | Value |
|---|---:|
| Rest angle | 90° |
| Kick angle | 0° |
| Movement time | 0.20 seconds |
| Hold time | 0.15 seconds |

The servo moves from its 90° rest position to 0° to kick a reject, then returns to 90°.

Test the servo with the conveyor and other machinery disconnected before normal operation. Confirm that the arm does not collide with the belt or frame.

---

## Running Manually

Stop the systemd service first to avoid sharing the camera and GPIO resources:

```bash
sudo systemctl stop cacao-sorter.service
```

Activate the environment:

```bash
cd ~/cacao_pulp
source venv313/bin/activate
```

Run:

```bash
python main.py
```

To show the live camera preview:

```bash
SHOW_PREVIEW=1 python main.py
```

A display or VNC session is required for the preview.

Press **Q** in the preview window or **Ctrl+C** in the terminal to stop the application.

Restart the service when finished:

```bash
sudo systemctl start cacao-sorter.service
```

---

## Pin State Check

Use `pinctrl` to inspect the configured GPIO pins:

```bash
pinctrl get 2
pinctrl get 3
pinctrl get 5
pinctrl get 12
pinctrl get 17
pinctrl get 23
pinctrl get 24
pinctrl get 25
pinctrl get 27
```

---

## Troubleshooting

| Symptom | Possible fix |
|---|---|
| LCD shows "Init Failed" | Check `~/cacao_pulp/error.log`, I2C configuration, and LCD wiring. |
| LCD blank or address `0x27` missing | Run `i2cdetect -y 1`; check SDA, SCL, VCC, and GND. |
| Could not open webcam | Stop the service and check `sudo lsof /dev/video0`. |
| Camera works manually but not at boot | Increase `ExecStartPre` delay and reload systemd. |
| Conveyor does not turn on | Check relay wiring, motor supply, and active-LOW configuration in `components/motors.py`. |
| Cracker or dispenser does not operate | Check GPIO24/GPIO25 wiring, relay configuration, and motor power. |
| Servo does not kick correctly | Check GPIO12, separate 5 V supply, common GND, and mechanical clearance. |
| Safe Shutdown button does nothing | Check GPIO17 wiring, the 2-second hold behavior, and sudoers permission. Confirm the GPIO17 shutdown overlay is not enabled. |
| Emergency stop does not respond | Check GPIO5 wiring and active-LOW input handling. Remember that this is software-controlled, not a hardware safety disconnect. |
| Model fails to load | Check `MODEL_PATH`, model filename, virtual environment, and LiteRT installation. |
| Detection is inaccurate | Verify the loaded V3 model, 416×416 input configuration, camera crop, lighting, and confidence threshold. |
| Script exits immediately | Inspect `~/cacao_pulp/error.log` and `sudo journalctl -u cacao-sorter.service`. |
| System returns to IDLE unexpectedly | Check confirmed detection counts, the 20-second empty-window timer, camera recovery, and fallback timeout settings. |

---

## Model and Dataset

The current deployed model is **V3**.

The training dataset uses two classes:

| Class ID | Class |
|---:|---|
| 0 | Good |
| 1 | Reject |

The V3 dataset contains:

- 1,003 images total.
- 751 training images.
- 252 validation images.
- 416×416 training images.
- Additional empty-belt background images.

V3 training results:

| Metric | Result |
|---|---:|
| mAP50 | 0.993 |
| mAP50-95 | 0.826 |
| Good precision | 0.953 |
| Good recall | 1.000 |
| Reject precision | 0.986 |
| Reject recall | 0.955 |

These are dataset validation results, not guaranteed real-world sorting accuracy. The validation set is not an independent production test set, and live conveyor performance still needs to be measured.

---

## Project Status and Testing

The V3 model has been exported to TFLite and imported into the Raspberry Pi application.

The updated belt-stage empty-window logic and current servo angles have been added to the project code, but the latest changes still require hardware testing.

Before production operation, verify:

- [ ] Correct V3 model loads on startup.
- [ ] Camera crop and model input are both configured for 416×416.
- [ ] Conveyor, cracker, and dispenser relays operate correctly.
- [ ] Servo returns to its 90° rest position.
- [ ] Confirmed good and reject detections update the counts correctly.
- [ ] Pending detections do not reset the 20-second timer.
- [ ] Confirmed detections reset the timer.
- [ ] The system returns to IDLE after 20 seconds without confirmed pulp.
- [ ] Camera recovery restarts the full 20-second window.
- [ ] Camera/model-unavailable fallback timeout works.
- [ ] Safe Shutdown works with a 2-second hold.
- [ ] Emergency-stop behavior is tested in every operating state.
- [ ] All relays turn off during shutdown and error handling.

**Do not treat the system as production-ready until the latest software changes have been tested on the actual hardware.**

---

## Repository

GitHub:

https://github.com/gabgonzales-ph/Cacao-Pulp-Detection-and-Sorting-Automation/