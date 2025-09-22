# camera_utils.py
import time
import platform
import cv2

# Probe tuning so we give USB cameras a moment to warm up
_PROBE_ATTEMPTS = 5
_PROBE_DELAY_S = 0.15

# Backends by OS
_BACKENDS = {
    "Darwin": [cv2.CAP_AVFOUNDATION, cv2.CAP_ANY],          # macOS
    "Linux": [cv2.CAP_V4L2, cv2.CAP_ANY],                   # Ubuntu etc.
    "Windows": [cv2.CAP_DSHOW, cv2.CAP_MSMF, cv2.CAP_ANY],  # Windows
}


def _os_key():
    sysname = platform.system()
    if sysname not in _BACKENDS:
        return "Linux"  # sensible default (also used for WSL)
    return sysname


def _can_read(cap):
    """Return True if the capture yields a frame within a few attempts."""
    for _ in range(_PROBE_ATTEMPTS):
        ok, frame = cap.read()
        if ok and frame is not None:
            return True
        time.sleep(_PROBE_DELAY_S)
    return False


def list_cameras(max_index=8, backends=None, warmup_ms=150):
    """
    Probe camera indices (0..max_index-1) and return those that open.
    Uses OS-appropriate backends in order. Returns a list of (index, backend).
    """
    key = _os_key()
    backends = backends or _BACKENDS[key]
    found = []
    for idx in range(max_index):
        for backend in backends:
            cap = cv2.VideoCapture(idx, backend)
            if not cap.isOpened():
                cap.release()
                continue

            time.sleep(warmup_ms / 1000.0)
            ok = _can_read(cap)
            cap.release()
            if ok:
                found.append((idx, backend))
                break  # don’t test other backends for this index
    return found


def open_camera(
    preferred_index=None,
    width=None,
    height=None,
    fps=None,
    backends=None,
    verbose=True,
):
    """
    Open a camera with the best backend for your OS.
      - preferred_index: int or None (auto-pick first working if None)
      - width/height: request capture resolution (best-effort)
      - fps: request frame rate (best-effort)
    Returns: cv2.VideoCapture
    Raises: RuntimeError if none open.
    """
    key = _os_key()
    backends = backends or _BACKENDS[key]

    backend_names = {
        cv2.CAP_AVFOUNDATION: "AVFOUNDATION",
        cv2.CAP_V4L2: "V4L2",
        cv2.CAP_DSHOW: "DSHOW",
        cv2.CAP_MSMF: "MSMF",
        cv2.CAP_ANY: "ANY",
    }

    def _try_open(idx):
        for backend in backends:
            cap = cv2.VideoCapture(idx, backend)
            if not cap.isOpened():
                cap.release()
                continue

            if width is not None:
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(width))
            if height is not None:
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(height))
            if fps is not None:
                cap.set(cv2.CAP_PROP_FPS, float(fps))

            if _can_read(cap):
                if verbose:
                    name = backend_names.get(backend, str(backend))
                    print(
                        "[camera_utils] Opened index "
                        f"{idx} using backend {name}"
                    )
                return cap

            cap.release()
        return None

    if preferred_index is not None:
        cap = _try_open(preferred_index)
        if cap is not None:
            return cap
        if verbose:
            message = (
                "[camera_utils] Preferred index "
                f"{preferred_index} failed; trying auto-scan…"
            )
            print(message)

    detected = list_cameras(max_index=8, backends=backends)
    if not detected:
        raise RuntimeError(
            "No camera detected.\n"
            "- macOS: enable Terminal/Python under Privacy & Security → "
            "Camera.\n"
            "- Linux: install v4l-utils/libgl1/ffmpeg (e.g. sudo apt install "
            "v4l-utils libgl1 ffmpeg) and verify /dev/video* permissions."
        )

    idx, _ = detected[0]
    cap = _try_open(idx)
    if cap is not None:
        return cap
    raise RuntimeError(
        "A camera was detected but could not be opened with any backend."
    )
