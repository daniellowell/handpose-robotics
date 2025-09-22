
# uHand controller via Hand Pose Detection

## Dependencies

Work in a virtual environment for better containment

```
python -m venv <my_venv_name>
source ./<my_venv_name>/bin/activate
```

Install the Python requirements
```
pip install -r requirements.txt
```

There may be other system-wide dependencies you are missing, not providing instruction on those.

Install the handpose model
```
# Using wget
wget https://storage.googleapis.com/mediapipe-models/hand_landmarker/hand_landmarker/float16/1/hand_landmarker.task

# OR
# Using curl
curl -o hand_landmarker.task https://storage.googleapis.com/mediapipe-models/hand_landmarker/hand_landmarker/float16/1/hand_landmarker.task
```

## Tests
```
# Tests the camera link is working
python ./test_camera_preview.py

# Tests the hand pose model is working, requires passing camera test
python ./test_mp_handpose.py

# Tests serial connection
python ./test_serial
```

## Running
```
# Main
python ./controller_mp312.py
```

Tests
